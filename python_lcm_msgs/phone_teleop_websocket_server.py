#!/usr/bin/env python3
# WebSocket server with fixed API compatibility

import asyncio
import json
import websockets
import argparse
import time
import ssl
import os
import logging
import traceback
from datetime import datetime
from colorama import init, Fore, Style
import lcm
from geometry_msgs import Twist

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler()]
)
logger = logging.getLogger('websocket_server')

# Initialize colorama for colored terminal output
init()

# Global variables for data display options
display_raw = False
display_motion = True
display_orientation = True
last_update_time = 0
update_interval = 0.1  # How often to update the terminal display (seconds)

class SensorDataReceiver:
    def __init__(self, host='localhost', port=8765):
        self.host = host
        self.port = port
        self.connections = set()
        self.last_data = None
        self.data_count = 0
        self.start_time = time.time()
        self.lc = lcm.LCM()

    async def register(self, websocket):
        """Register a new client connection"""
        client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        self.connections.add(websocket)
        print(f"{Fore.GREEN}[+] New client connected: {client_info}{Style.RESET_ALL}")
        logger.info(f"New client connected: {client_info}")
        print(f"   Total clients: {len(self.connections)}")
        
        # Send an acknowledgment message
        try:
            await websocket.send(json.dumps({
                "type": "welcome",
                "message": "Connected to sensor data server",
                "timestamp": int(time.time() * 1000)
            }))
        except Exception as e:
            logger.error(f"Error sending welcome message: {e}")
        
    async def unregister(self, websocket):
        """Unregister a client connection"""
        if websocket in self.connections:
            self.connections.remove(websocket)
            client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
            print(f"{Fore.YELLOW}[-] Client disconnected: {client_info}{Style.RESET_ALL}")
            logger.info(f"Client disconnected: {client_info}")
            print(f"   Total clients: {len(self.connections)}")
    
    # Fixed handler function that works with newer websockets versions
    async def handle_connection(self, websocket):
        """Handle a client connection"""
        try:
            await self.register(websocket)
            
            async for message in websocket:
                try:
                    await self.process_message(websocket, message)
                except Exception as e:
                    error_msg = f"Error processing message: {e}"
                    logger.error(error_msg)
                    logger.debug(traceback.format_exc())
                    try:
                        await websocket.send(json.dumps({
                            "type": "error",
                            "message": str(e)
                        }))
                    except:
                        pass  # Ignore errors in sending error messages
        except websockets.exceptions.ConnectionClosed as e:
            logger.info(f"Connection closed: {e.code} - {e.reason}")
            print(f"{Fore.RED}[!] Connection closed: Code {e.code} - {e.reason}{Style.RESET_ALL}")
        except Exception as e:
            logger.error(f"Error in connection handler: {e}")
            logger.debug(traceback.format_exc())
            print(f"{Fore.RED}[!] Error in connection handler: {e}{Style.RESET_ALL}")
        finally:
            await self.unregister(websocket)
            
    async def process_message(self, websocket, message):
        """Process incoming WebSocket messages"""
        try:
            data = json.loads(message)
            
            # Handle ping messages for heartbeat
            if data.get("type") == "ping":
                await websocket.send(json.dumps({
                    "type": "pong",
                    "timestamp": int(time.time() * 1000)
                }))
                return
                
            # Skip non-sensor data messages
            if data.get("type") != "sensorData" and "position" not in data:
                return
                
            self.last_data = data
            self.data_count += 1
            
            # Print data at a controlled rate to avoid flooding the terminal
            current_time = time.time()
            global last_update_time
            
            if current_time - last_update_time >= update_interval:
                last_update_time = current_time
                self.display_data(data)
                
        except json.JSONDecodeError:
            logger.warning("Received invalid JSON data")
            print(f"{Fore.RED}[!] Error: Received invalid JSON data{Style.RESET_ALL}")
            await websocket.send(json.dumps({
                "type": "error",
                "message": "Invalid JSON format"
            }))
    
    def display_data(self, data):
        """Display the received sensor data"""
        # Clear terminal (uncomment if desired)
        # print("\033c", end="")
        
        # Print header
        data_rate = self.data_count / (time.time() - self.start_time)
        timestamp = datetime.fromtimestamp(data.get('timestamp', time.time()*1000)/1000).strftime('%H:%M:%S.%f')[:-3]
        
        print(f"\n{Fore.CYAN}══════════════════════════════════════════════════{Style.RESET_ALL}")
        print(f"{Fore.CYAN}Sensor Data - {timestamp} - Rate: {data_rate:.1f} msg/s{Style.RESET_ALL}")
        print(f"{Fore.CYAN}══════════════════════════════════════════════════{Style.RESET_ALL}")
        
        # Display position data
        if display_motion and 'position' in data:
            print(f"{Fore.GREEN}Position:{Style.RESET_ALL}")
            print(f"  X: {data['position']['x']:.4f} m")
            print(f"  Y: {data['position']['y']:.4f} m")
            print(f"  Z: {data['position']['z']:.4f} m")
            
            print(f"{Fore.GREEN}Velocity:{Style.RESET_ALL}")
            print(f"  X: {data['velocity']['x']:.4f} m/s")
            print(f"  Y: {data['velocity']['y']:.4f} m/s")
            print(f"  Z: {data['velocity']['z']:.4f} m/s")
        
        # Display orientation data
        if display_orientation and 'orientation' in data:
            print(f"{Fore.GREEN}Orientation:{Style.RESET_ALL}")
            print(f"  Roll:  {data['orientation']['roll']:.2f}°")
            print(f"  Pitch: {data['orientation']['pitch']:.2f}°")
            print(f"  Yaw:   {data['orientation']['yaw']:.2f}°")

        msg = Twist()
        msg.linear.x = data['position']['x']
        msg.linear.y = data['position']['y']
        msg.linear.z = data['position']['z']
        msg.angular.x = data['orientation']['roll']
        msg.angular.y = data['orientation']['pitch']
        msg.angular.z = data['orientation']['yaw']
        self.lc.publish("TWIST", msg.encode())
        
        # Display raw accelerometer/gyroscope data if requested
        if display_raw:
            if 'accelerometer' in data:
                print(f"{Fore.GREEN}Accelerometer:{Style.RESET_ALL}")
                print(f"  X: {data['accelerometer']['x']:.4f} m/s²")
                print(f"  Y: {data['accelerometer']['y']:.4f} m/s²")
                print(f"  Z: {data['accelerometer']['z']:.4f} m/s²")
            
            if 'gyroscope' in data:
                print(f"{Fore.GREEN}Gyroscope:{Style.RESET_ALL}")
                print(f"  X: {data['gyroscope']['x']:.4f} °/s")
                print(f"  Y: {data['gyroscope']['y']:.4f} °/s")
                print(f"  Z: {data['gyroscope']['z']:.4f} °/s")
        
        # Display stationary status
        if 'isStationary' in data:
            stationary_status = "Yes" if data['isStationary'] else "No"
            color = Fore.GREEN if data['isStationary'] else Fore.RED
            print(f"{Fore.GREEN}Stationary:{Style.RESET_ALL} {color}{stationary_status}{Style.RESET_ALL}")
    
    async def start_server(self, ssl_context=None):
        """Start the WebSocket server"""
        protocol = "wss" if ssl_context else "ws"
        print(f"{Fore.CYAN}Starting {protocol}:// WebSocket server on {self.host}:{self.port}{Style.RESET_ALL}")
        print(f"{Fore.CYAN}Press Ctrl+C to exit{Style.RESET_ALL}")
        
        # Create server with appropriate SSL context
        try:
            server = await websockets.serve(
                self.handle_connection, 
                self.host, 
                self.port, 
                ssl=ssl_context,
                ping_interval=30,  # Send pings every 30 seconds
                ping_timeout=10,    # Wait 10 seconds for pong response
                max_size=None       # No message size limit
            )
            self.start_time = time.time()
            logger.info(f"Server started on {self.host}:{self.port} ({'secure' if ssl_context else 'insecure'})")
            return server
        except Exception as e:
            logger.error(f"Failed to start server: {e}")
            print(f"{Fore.RED}[!] Failed to start server: {e}{Style.RESET_ALL}")
            if ssl_context:
                print(f"{Fore.YELLOW}Try running without SSL or check your certificate files.{Style.RESET_ALL}")
            raise

def generate_self_signed_cert(cert_path, key_path):
    """Generate a self-signed certificate if it doesn't exist"""
    if os.path.exists(cert_path) and os.path.exists(key_path):
        print(f"{Fore.GREEN}Using existing SSL certificate and key{Style.RESET_ALL}")
        return True
    
    print(f"{Fore.YELLOW}Generating self-signed SSL certificate...{Style.RESET_ALL}")
    
    # Make sure the directory exists
    cert_dir = os.path.dirname(cert_path)
    if cert_dir and not os.path.exists(cert_dir):
        os.makedirs(cert_dir)
    
    key_dir = os.path.dirname(key_path)
    if key_dir and not os.path.exists(key_dir):
        os.makedirs(key_dir)
    
    # Generate a self-signed certificate using OpenSSL
    # This requires OpenSSL to be installed on the system
    cmd = (
        f"openssl req -x509 -newkey rsa:4096 -keyout {key_path} -out {cert_path} "
        f"-days 365 -nodes -subj '/CN=localhost'"
    )
    
    import subprocess
    try:
        subprocess.run(cmd, shell=True, check=True)
        print(f"{Fore.GREEN}SSL certificate and key generated successfully{Style.RESET_ALL}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"{Fore.RED}Error generating SSL certificate: {e}{Style.RESET_ALL}")
        print(f"{Fore.RED}Please ensure OpenSSL is installed and try again{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}Continuing without SSL...{Style.RESET_ALL}")
        return False

async def main():
    """Main function to run the server"""
    parser = argparse.ArgumentParser(description='WebSocket Server for Sensor Data')
    parser.add_argument('--host', default='0.0.0.0', help='Host address to bind (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8765, help='Port to bind (default: 8765)')
    parser.add_argument('--raw', action='store_true', help='Display raw sensor data')
    parser.add_argument('--interval', type=float, default=0.1, help='Update interval in seconds (default: 0.1)')
    parser.add_argument('--ssl', action='store_true', help='Enable SSL/TLS (WSS) for secure WebSocket')
    parser.add_argument('--cert', default='./cert/server.crt', help='SSL certificate file path')
    parser.add_argument('--key', default='./cert/server.key', help='SSL key file path')
    parser.add_argument('--no-generate-cert', action='store_true', help='Do not generate a self-signed certificate')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging')
    parser.add_argument('--no-ssl', action='store_true', help='Force non-SSL mode (for testing)')
    args = parser.parse_args()
    
    # Set up debug logging if requested
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
        logger.setLevel(logging.DEBUG)
        logger.debug("Debug logging enabled")
    
    # Update global display settings
    global display_raw, update_interval
    display_raw = args.raw
    update_interval = args.interval
    
    # Set up SSL if requested
    ssl_context = None
    if args.ssl and not args.no_ssl:
        if not args.no_generate_cert and not (os.path.exists(args.cert) and os.path.exists(args.key)):
            generate_self_signed_cert(args.cert, args.key)
        
        if os.path.exists(args.cert) and os.path.exists(args.key):
            try:
                ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
                ssl_context.load_cert_chain(args.cert, args.key)
                print(f"{Fore.GREEN}SSL enabled with certificate: {args.cert}{Style.RESET_ALL}")
                logger.info(f"SSL enabled with certificate: {args.cert}")
            except Exception as e:
                print(f"{Fore.RED}Error loading SSL certificate: {e}{Style.RESET_ALL}")
                logger.error(f"Error loading SSL certificate: {e}")
                print(f"{Fore.YELLOW}Continuing without SSL...{Style.RESET_ALL}")
                ssl_context = None
        else:
            print(f"{Fore.RED}SSL certificate or key not found: {args.cert}, {args.key}{Style.RESET_ALL}")
            logger.error(f"SSL certificate or key not found: {args.cert}, {args.key}")
            print(f"{Fore.YELLOW}Continuing without SSL...{Style.RESET_ALL}")
    
    # Create and start the WebSocket server
    receiver = SensorDataReceiver(args.host, args.port)
    try:
        server = await receiver.start_server(ssl_context)
        
        print(f"\n{Fore.GREEN}=== SERVER READY ==={Style.RESET_ALL}")
        print(f"Connect to: {'wss' if ssl_context else 'ws'}://{args.host}:{args.port}")
        if args.host == '0.0.0.0':
            import socket
            try:
                hostname = socket.gethostname()
                local_ip = socket.gethostbyname(hostname)
                print(f"Local IP: {'wss' if ssl_context else 'ws'}://{local_ip}:{args.port}")
            except:
                pass
            
        await asyncio.Future()  # Run forever
    except KeyboardInterrupt:
        print(f"\n{Fore.YELLOW}Server shutdown requested...{Style.RESET_ALL}")
    except Exception as e:
        print(f"{Fore.RED}Server error: {e}{Style.RESET_ALL}")
        logger.error(f"Server error: {e}")
        logger.debug(traceback.format_exc())
    finally:
        if 'server' in locals():
            server.close()
            await server.wait_closed()
            print(f"{Fore.YELLOW}Server stopped{Style.RESET_ALL}")
            logger.info("Server stopped")

if __name__ == "__main__":
    asyncio.run(main())