import dash
from dash import dcc, html
import plotly.graph_objs as go
import zmq
import json
import time
import threading
import queue

# Create a thread-safe queue for data exchange
data_queue = queue.Queue(maxsize=1)

class DataReceiver:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5555")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages
        self.running = True
        
        # Data storage
        self.sensor_data_window = []
        self.cluster_displacements = {"Middle": [], "Top": [], "Bottom Right": [], "Bottom Left": []}
        self.cluster_torques = {"Middle": [], "Top": [], "Bottom Right": [], "Bottom Left": []}
        self.forces = []
        self.torques = []
        self.calibrated = False
        
    def receive_data(self):
        """Continuously receive data from the acquisition process"""
        while self.running:
            try:
                # Non-blocking receive with timeout
                if self.socket.poll(100):  # 100ms timeout
                    data_dict = self.socket.recv_json()
                    
                    # Update local data copies
                    self.sensor_data_window = data_dict.get('sensor_data_window', [])
                    self.cluster_displacements = data_dict.get('cluster_displacements', self.cluster_displacements)
                    self.cluster_torques = data_dict.get('cluster_torques', self.cluster_torques)
                    self.forces = data_dict.get('forces', [])
                    self.torques = data_dict.get('torques', [])
                    self.calibrated = data_dict.get('calibrated', False)
                    
                    # Put the latest data in the queue (replacing old data if full)
                    if data_queue.full():
                        data_queue.get_nowait()  # Remove old data
                    data_queue.put_nowait(data_dict)
            except zmq.ZMQError as e:
                print(f"ZMQ error: {e}")
                time.sleep(0.1)
            except Exception as e:
                print(f"Error receiving data: {e}")
                time.sleep(0.1)
    
    def get_latest_data(self):
        """Get the latest data received from the queue"""
        try:
            if not data_queue.empty():
                data = data_queue.get_nowait()
                return data
            return None
        except queue.Empty:
            return None
    
    def stop(self):
        """Stop the data receiver"""
        self.running = False
        self.socket.close()
        self.context.term()

# Create Dash application
app = dash.Dash(__name__)
cluster_names = ["Middle", "Top", "Bottom Right", "Bottom Left"]

app.layout = html.Div([
    html.H1("Real-time Sensor Data & Cluster Displacements"),
    dcc.Graph(id="raw-sensor-graph"),
    html.Div([
        html.Div([dcc.Graph(id=f"{name}-graph")], style={'width': '48%', 'display': 'inline-block'}) 
        for name in cluster_names
    ], style={'display': 'flex', 'flex-wrap': 'wrap'}),
    html.Div([
        html.Div([dcc.Graph(id=f"{name}-torque-graph")], style={'width': '48%', 'display': 'inline-block'}) 
        for name in cluster_names
    ], style={'display': 'flex', 'flex-wrap': 'wrap'}),
    html.Div([
        html.Div([dcc.Graph(id=f"force-graph")], style={'width': '48%', 'display': 'inline-block'}),
        html.Div([dcc.Graph(id=f"torque-graph")], style={'width': '48%', 'display': 'inline-block'}),
    ], style={'display': 'flex', 'flex-wrap': 'wrap'}),
    dcc.Interval(id="interval-component", interval=100, n_intervals=0),
    html.Div(id="data-store", style={'display': 'none'})
])

# Data storage for the callbacks
global_data = {
    'sensor_data_window': [],
    'cluster_displacements': {"Middle": [], "Top": [], "Bottom Right": [], "Bottom Left": []},
    'cluster_torques': {"Middle": [], "Top": [], "Bottom Right": [], "Bottom Left": []},
    'forces': [],
    'torques': [],
    'calibrated': False
}

# Update data store from ZMQ
@app.callback(
    dash.dependencies.Output("data-store", "children"),
    [dash.dependencies.Input("interval-component", "n_intervals")]
)
def update_data_store(n):
    data = receiver.get_latest_data()
    if data:
        # Update global data
        global global_data
        global_data = data
    return json.dumps({'timestamp': time.time()})  # Just a trigger for other callbacks

@app.callback(
    dash.dependencies.Output("raw-sensor-graph", "figure"),
    [dash.dependencies.Input("data-store", "children")]
)
def update_raw_sensor_graph(json_data):
    if not global_data['sensor_data_window']:
        return go.Figure()
    
    x_vals = list(range(len(global_data['sensor_data_window'])))
    fig = go.Figure()
    
    for i in range(16):
        y_vals = [row[i] for row in global_data['sensor_data_window']]
        fig.add_trace(go.Scatter(x=x_vals, y=y_vals, mode="lines", name=f"Sensor {i+1}"))
    
    fig.update_layout(title="Raw Sensor Data", xaxis_title="Time", yaxis_title="Value")
    return fig

# Create dynamic callbacks for all clusters
for name in cluster_names:
    @app.callback(
        dash.dependencies.Output(f"{name}-graph", "figure"),
        [dash.dependencies.Input("data-store", "children")]
    )
    def update_displacement_graph(json_data, cluster=name):
        displacements = global_data['cluster_displacements'].get(cluster, [])
        if not displacements:
            return go.Figure()
        
        x_vals = list(range(len(displacements)))
        fig = go.Figure()
        
        # Transpose the list of displacements to get all x, y, z values
        try:
            xyz_displacements = list(zip(*displacements))
            for dim, axis in enumerate(["X", "Y", "Z"]):
                fig.add_trace(go.Scatter(x=x_vals, y=xyz_displacements[dim], mode="lines", name=f"{axis} Displacement"))
        except IndexError:
            # If there's not enough data yet
            pass
            
        fig.update_layout(title=f"{cluster} Cluster Displacements", xaxis_title="Time", yaxis_title="Displacement")
        return fig

# Create dynamic callbacks for all cluster torques
for name in cluster_names:
    @app.callback(
        dash.dependencies.Output(f"{name}-torque-graph", "figure"),
        [dash.dependencies.Input("data-store", "children")]
    )
    def update_torque_graph(json_data, cluster=name):
        torques = global_data['cluster_torques'].get(cluster, [])
        if not torques:
            return go.Figure()
        
        x_vals = list(range(len(torques)))
        fig = go.Figure()
        
        try:
            xyz_torques = list(zip(*torques))
            for dim, axis in enumerate(["X", "Y", "Z"]):
                fig.add_trace(go.Scatter(x=x_vals, y=xyz_torques[dim], mode="lines", name=f"{axis} Torque"))
        except IndexError:
            pass
            
        fig.update_layout(title=f"{cluster} Cluster Torques", xaxis_title="Time", yaxis_title="Torque")
        return fig

@app.callback(
    dash.dependencies.Output("force-graph", "figure"),
    [dash.dependencies.Input("data-store", "children")]
)
def update_force_graph(json_data):
    forces = global_data['forces']
    if not forces:
        return go.Figure()
    
    x_vals = list(range(len(forces)))
    fig = go.Figure()
    
    try:
        xyz_forces = list(zip(*forces))
        for dim, axis in enumerate(["X", "Y", "Z"]):
            fig.add_trace(go.Scatter(x=x_vals, y=xyz_forces[dim], mode="lines", name=f"{axis} Force"))
    except IndexError:
        pass
        
    fig.update_layout(title="Force", xaxis_title="Time", yaxis_title="Force")
    return fig

@app.callback(
    dash.dependencies.Output("torque-graph", "figure"),
    [dash.dependencies.Input("data-store", "children")]
)
def update_torque_graph(json_data):
    torques = global_data['torques']
    if not torques:
        return go.Figure()
    
    x_vals = list(range(len(torques)))
    fig = go.Figure()
    
    try:
        xyz_torques = list(zip(*torques))
        for dim, axis in enumerate(["X", "Y", "Z"]):
            fig.add_trace(go.Scatter(x=x_vals, y=xyz_torques[dim], mode="lines", name=f"{axis} Torque"))
    except IndexError:
        pass
        
    fig.update_layout(title="Torque", xaxis_title="Time", yaxis_title="Torque")
    return fig

if __name__ == "__main__":
    # Start the data receiver in a separate thread
    receiver = DataReceiver()
    receiver_thread = threading.Thread(target=receiver.receive_data, daemon=True)
    receiver_thread.start()
    
    try:
        # Run the Dash app
        app.run_server(debug=False)  # Set debug=False in production to avoid multiple threads
    finally:
        # Clean up
        receiver.stop()
        receiver_thread.join(timeout=1.0)
        print("Visualization stopped.")