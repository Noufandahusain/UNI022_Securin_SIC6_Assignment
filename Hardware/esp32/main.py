from machine import Pin, I2C, UART, Timer
import network
import time
import ujson
import math
import ufirebase as firebase
import gc
from umqtt.robust import MQTTClient  # Add MQTT client for Ubidots

# Configuration
WIFI_SSID = "Realme GT Neo 3"
WIFI_PASSWORD = "qwertyuiop1"

# Firebase Configuration
FIREBASE_URL = "https://muqsithfirebase-default-rtdb.asia-southeast1.firebasedatabase.app/"  # Ganti dengan URL Firebase Anda
FIREBASE_PATH = "/esp32_accident_detection"  # Path untuk menyimpan data

# Ubidots Configuration
UBIDOTS_TOKEN = "BBUS-2Ql745IrLGQ2jbhqWe1iRO9KHxjFqI"
UBIDOTS_DEVICE_LABEL = "esp32"
ubidots_client = None  # Will be initialized later

# Pin definitions
# I2C for MPU6050
I2C_SCL_PIN = 22  # GPIO22
I2C_SDA_PIN = 21  # GPIO21

# UART for GPS module
GPS_TX_PIN = 17  # Connect to RX of ESP32
GPS_RX_PIN = 16  # Connect to TX of ESP32

# MPU6050 constants
MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# System states
STATE_INIT = 0
STATE_PARK = 1
STATE_DRIVE = 2
STATE_ACCIDENT = 3
current_state = STATE_INIT
state_names = ["INIT", "PARK", "DRIVE", "ACCIDENT"]

# Accident detection configuration
ACCIDENT_ACCEL_THRESHOLD = 1.5  # g-force threshold (adjust based on testing)
ACCIDENT_GYRO_THRESHOLD = 200.0  # degrees/s threshold (adjust based on testing)
ACCIDENT_DETECTION_WINDOW = 5    # Number of samples to consider
accident_buffer = []             # Buffer to store recent acceleration data
accident_cooldown = 0            # Cooldown timer after accident detection
ACCIDENT_COOLDOWN_PERIOD = 30    # Seconds to wait before allowing new accident detection

# Firebase rate limiting
FIREBASE_SEND_INTERVAL = 20      # Send data every 20 seconds
last_data_send = 0               # Last time data was sent
CRITICAL_SEND_INTERVAL = 5       # Send critical data every 5 seconds

# Initialize all variables
i2c = None
gps_uart = None
data_timer = None
gps_clock = None

# GPS data
gps_lat = 0.0
gps_lon = 0.0
gps_valid = False
gps_time = None
last_movement_time = 0

# Function to safely initialize hardware
def initialize_hardware():
    global i2c, gps_uart
    
    try:
        # Initialize I2C for MPU6050
        i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=400000)
        
        # Initialize UART for GPS
        gps_uart = UART(2, baudrate=9600, tx=Pin(GPS_TX_PIN), rx=Pin(GPS_RX_PIN))
        
        return True
    except Exception as e:
        print(f"Hardware initialization error: {e}")
        return False

# Function to connect to WiFi
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    print("Connecting to WiFi...")
    
    max_retries = 3
    for attempt in range(max_retries):
        try:
            if not wlan.isconnected():
                wlan.connect(WIFI_SSID, WIFI_PASSWORD)
                
                timeout = 20
                while not wlan.isconnected() and timeout > 0:
                    gc.collect()
                    print(f"Waiting for connection... {timeout}s")
                    time.sleep(1)
                    timeout -= 1
            
            if wlan.isconnected():
                print('WiFi connected!')
                print('Network config:', wlan.ifconfig())
                return True
            else:
                print(f'WiFi connection failed (attempt {attempt+1}/{max_retries})')
                time.sleep(2)
        
        except OSError as e:
            print(f"WiFi connection error: {e}")
            print(f"Retrying ({attempt+1}/{max_retries})...")
            wlan.active(False)
            time.sleep(2)
            wlan.active(True)
            time.sleep(1)
    
    print("All WiFi connection attempts failed")
    return False

# Initialize Firebase
def init_firebase():
    try:
        firebase.setURL(FIREBASE_URL)
        print("Firebase initialized")
        return True
    except Exception as e:
        print(f"Firebase initialization error: {e}")
        return False

# Initialize Ubidots
def init_ubidots():
    global ubidots_client
    
    try:
        client_id = f"esp32_{time.time()}"  # Generate unique client ID
        ubidots_client = MQTTClient(client_id, "industrial.api.ubidots.com", 1883, 
                                  user=UBIDOTS_TOKEN, password=UBIDOTS_TOKEN)
        ubidots_client.connect()
        print("Ubidots initialized")
        return True
    except Exception as e:
        print(f"Ubidots initialization error: {e}")
        return False

# Function to send data to Firebase
def send_to_firebase(data_dict, is_critical=False):
    try:
        # Add timestamp for each record
        data_dict["timestamp"] = gps_clock
        
        path = FIREBASE_PATH
        if is_critical:
            path += "/critical"
        else:
            path += "/status"
            
        firebase.put(path, data_dict, bg=False)  # Use background=False for critical data
        print(f"Data sent to Firebase: {path}")
        return True
    except Exception as e:
        print(f"Error sending data to Firebase: {e}")
        return False

# Function to send data to Ubidots
def send_to_ubidots(data_dict, is_critical=False):
    
    global ubidots_client
    
    if ubidots_client is None:
        print("Ubidots client not initialized")
        return False
    
    try:
        # Format data for Ubidots
        topic = f"/v1.6/devices/{UBIDOTS_DEVICE_LABEL}"
        
        # Create a flattened dictionary for Ubidots
        ubidots_data = {}
        
        # Add location data if available
        if "location" in data_dict:
            if data_dict["location"]["valid"]:
                ubidots_data["location"] = {
                    "value": 1,  # Need a value for the variable
                    "context": {
                        "lat": data_dict["location"]["lat"],
                        "lng": data_dict["location"]["lon"]
                    }
                }
        
        # Add sensor data if available
        if "sensor" in data_dict:
            if "accelerometer" in data_dict["sensor"]:
                ubidots_data["accel_x"] = {"value": data_dict["sensor"]["accelerometer"]["x"]}
                ubidots_data["accel_y"] = {"value": data_dict["sensor"]["accelerometer"]["y"]}
                ubidots_data["accel_z"] = {"value": data_dict["sensor"]["accelerometer"]["z"]}
                ubidots_data["accel_magnitude"] = {"value": data_dict["sensor"]["accelerometer"]["magnitude"]}
            
            if "gyroscope" in data_dict["sensor"]:
                ubidots_data["gyro_x"] = {"value": data_dict["sensor"]["gyroscope"]["x"]}
                ubidots_data["gyro_y"] = {"value": data_dict["sensor"]["gyroscope"]["y"]}
                ubidots_data["gyro_z"] = {"value": data_dict["sensor"]["gyroscope"]["z"]}
                ubidots_data["gyro_magnitude"] = {"value": data_dict["sensor"]["gyroscope"]["magnitude"]}
        
        # Add system state if available
        if "system" in data_dict:
            if "state" in data_dict["system"]:
                # Convert state to number for Ubidots
                state_value = current_state  # Use the current state enum value
                ubidots_data["system_state"] = {"value": state_value}
            
            if "movement" in data_dict["system"]:
                movement_value = 1 if data_dict["system"]["movement"] else 0
                ubidots_data["movement"] = {"value": movement_value}
        
        # Add accident data if available
        if "event" in data_dict and data_dict["event"] == "accident":
            ubidots_data["accident"] = {"value": 1}
            if "severity" in data_dict:
                ubidots_data["accident_severity"] = {"value": data_dict["severity"]}
        
        # Convert to JSON and publish
        payload = ujson.dumps(ubidots_data)
        ubidots_client.publish(topic, payload)
        time.sleep(2)
        print(f"Data sent to Ubidots: {topic}")
        
        return True
    except Exception as e:
        print(f"Error sending data to Ubidots: {e}")
        
        # Try to reconnect Ubidots client
        try:
            ubidots_client.disconnect()
            time.sleep(1)
            init_ubidots()
        except:
            pass
            
        return False

# Initialize MPU6050
def init_mpu6050():
    global i2c
    
    if i2c is None:
        print("I2C not initialized")
        return False
    
    try:
        # Wake up MPU6050
        i2c.writeto_mem(MPU_ADDR, PWR_MGMT_1, b'\x00')
        time.sleep(0.1)
        
        # Check if MPU6050 is responding
        if MPU_ADDR not in i2c.scan():
            print("MPU6050 not found!")
            return False
            
        print("MPU6050 initialized")
        return True
    except Exception as e:
        print("Failed to initialize MPU6050:", e)
        return False

# Read acceleration data from MPU6050
def read_mpu6050():
    global i2c
    
    if i2c is None:
        print("I2C not initialized in read_mpu6050")
        return {
            "accel_x": 0, "accel_y": 0, "accel_z": 0,
            "gyro_x": 0, "gyro_y": 0, "gyro_z": 0
        }
    
    try:
        # Read 14 bytes of data starting from ACCEL_XOUT_H register
        data = i2c.readfrom_mem(MPU_ADDR, ACCEL_XOUT_H, 14)
        
        # Helper function to convert the data
        def conv(high, low):  # helper to convert to signed
            val = (high << 8) | low
            return val - 65536 if val > 32767 else val
            
        # Convert the data
        accel_x = conv(data[0], data[1]) / 16384.0  # Convert to g (±2g scale)
        accel_y = conv(data[2], data[3]) / 16384.0
        accel_z = conv(data[4], data[5]) / 16384.0
        
        # Skip temperature (data[6], data[7])
        
        gyro_x = conv(data[8], data[9]) / 131.0  # Convert to degrees/s (±250°/s scale)
        gyro_y = conv(data[10], data[11]) / 131.0
        gyro_z = conv(data[12], data[13]) / 131.0
        
        return {
            "accel_x": accel_x, 
            "accel_y": accel_y, 
            "accel_z": accel_z,
            "gyro_x": gyro_x, 
            "gyro_y": gyro_y, 
            "gyro_z": gyro_z
        }
    except Exception as e:
        print("Failed to read MPU6050:", e)
        return {
            "accel_x": 0, "accel_y": 0, "accel_z": 0,
            "gyro_x": 0, "gyro_y": 0, "gyro_z": 0
        }

# Parse NMEA data from GPS
def parse_gps_data(line):
    global gps_lat, gps_lon, gps_valid, gps_clock
    
    try:
        if b"$GPRMC" in line:
            parts = line.decode('ascii').split(',')
            if len(parts) >= 10 and parts[2] == 'A':  # 'A' means data is valid
                # Parse latitude
                gps_clock = parts[1]
                print(gps_clock)
                if parts[3] and parts[4]:
                    lat_deg = float(parts[3][:2])
                    lat_min = float(parts[3][2:])
                    lat = lat_deg + lat_min / 60.0
                    if parts[4] == 'S':
                        lat = -lat
                    
                    # Parse longitude
                    lon_deg = float(parts[5][:3])
                    lon_min = float(parts[5][3:])
                    lon = lon_deg + lon_min / 60.0
                    if parts[6] == 'W':
                        lon = -lon
                    
                    gps_lat = lat
                    gps_lon = lon
                    gps_valid = True
                    return True
        return False
    except Exception as e:
        print("GPS parsing error:", e)
        return False

# Read GPS data
def read_gps(timer=None):
    global gps_uart, gps_valid
    
    if gps_uart is None:
        print("GPS UART not initialized in read_gps")
        return None
    
    try:
        if gps_uart.any():
            buffer = b""
            # Read all available data
            while gps_uart.any():
                gc.collect()
                buffer += gps_uart.readline()
                time.sleep(0.05)
            
            # Process each line
            lines = buffer.split(b'\r\n')
            for line in lines:
                if line:  # Skip empty lines
                    if parse_gps_data(line):
                        return True
    except Exception as e:
        print("Error reading GPS:", e)
    
    return None

# Detect accident based on sensor data
def detect_accident(sensor_data):
    global accident_buffer, accident_cooldown
    
    # Check if we're in cooldown period
    current_time = time.time()
    if current_time < accident_cooldown:
        return False
    
    # Add new data to buffer
    accident_buffer.append(sensor_data)
    
    # Keep buffer at specified size
    if len(accident_buffer) > ACCIDENT_DETECTION_WINDOW:
        accident_buffer.pop(0)
    
    # Need enough samples to make a decision
    if len(accident_buffer) < ACCIDENT_DETECTION_WINDOW:
        return False
    
    # Analyze buffer for accident patterns
    for data in accident_buffer:
        # Calculate magnitudes
        accel_magnitude = math.sqrt(data["accel_x"]**2 + data["accel_y"]**2 + data["accel_z"]**2)
        gyro_magnitude = math.sqrt(data["gyro_x"]**2 + data["gyro_y"]**2 + data["gyro_z"]**2)
        
        # Check for sudden peaks that indicate accident
        if accel_magnitude > ACCIDENT_ACCEL_THRESHOLD or gyro_magnitude > ACCIDENT_GYRO_THRESHOLD:
            # Clear buffer after detection
            accident_buffer = []
            
            # Set cooldown period
            accident_cooldown = current_time + ACCIDENT_COOLDOWN_PERIOD
            
            return True
    
    return False

# Handle accident event - priority data, always send
def handle_accident():
    global current_state, gps_lat, gps_lon, gps_valid
    
    print("ACCIDENT DETECTED! Sending alert...")
    
    # Create a clean, structured JSON for the accident event
    accident_data = {
        "event": "accident",
        "location": {
            "lat": gps_lat,
            "lon": gps_lon,
            "valid": gps_valid
        },
        "severity": 3,  # Scale 1-3, 3 being high
        "state": "ACCIDENT"
    }
    
    # Accident data is critical, so send it immediately
    if send_to_firebase(accident_data, is_critical=True):
        print("Accident alert sent to Firebase successfully")
    else:
        print("Failed to send accident alert to Firebase, retrying...")
        # Try again after a short delay
        time.sleep(1)
        send_to_firebase(accident_data, is_critical=True)
    
    # Send to Ubidots as well
    if send_to_ubidots(accident_data, is_critical=True):
        print("Accident alert sent to Ubidots successfully")
    else:
        print("Failed to send accident alert to Ubidots, retrying...")
        # Try again after a short delay
        time.sleep(1)
        send_to_ubidots(accident_data, is_critical=True)
    
    # Update system state to ACCIDENT
    current_state = STATE_ACCIDENT
    
    print("Accident handling complete")

# Function to send sensor data
def send_sensor_data(timer=None):
    global current_state, last_movement_time, accident_cooldown, last_data_send
    
    try:
        # Read MPU6050 data
        sensor_data = read_mpu6050()
        
        # Calculate magnitudes
        accel_magnitude = math.sqrt(sensor_data["accel_x"]**2 + sensor_data["accel_y"]**2 + sensor_data["accel_z"]**2)
        gyro_magnitude = math.sqrt(sensor_data["gyro_x"]**2 + sensor_data["gyro_y"]**2 + sensor_data["gyro_z"]**2)
        
        # Calculate change in acceleration (deviation from 1g at rest)
        accel_change = abs(accel_magnitude - 1.0)
        current_time = time.time()
        
        # Only send sensor data periodically to limit data volume
        if current_time - last_data_send >= FIREBASE_SEND_INTERVAL:
            # Create a clean, structured JSON with all the sensor data
            status_data = {
                "sensor": {
                    "accelerometer": {
                        "x": round(sensor_data["accel_x"], 3),
                        "y": round(sensor_data["accel_y"], 3),
                        "z": round(sensor_data["accel_z"], 3),
                        "magnitude": round(accel_magnitude, 3)
                    },
                    "gyroscope": {
                        "x": round(sensor_data["gyro_x"], 3),
                        "y": round(sensor_data["gyro_y"], 3),
                        "z": round(sensor_data["gyro_z"], 3),
                        "magnitude": round(gyro_magnitude, 3)
                    }
                },
                "location": {
                    "lat": gps_lat,
                    "lon": gps_lon,
                    "valid": gps_valid
                },
                "system": {
                    "state": state_names[current_state],
                    "movement": accel_change > 0.3 or gyro_magnitude > 15.0
                }
            }
            
            # Send to Firebase
            if send_to_firebase(status_data):
                print("Sensor data sent to Firebase successfully")
            else:
                print("Failed to send sensor data to Firebase, will retry later")
            
            # Send to Ubidots
            if send_to_ubidots(status_data):
                print("Sensor data sent to Ubidots successfully")
            else:
                print("Failed to send sensor data to Ubidots, will retry later")
                
            # Update last send time
            last_data_send = current_time
        
        # State transitions based on current state
        if current_state == STATE_PARK:
            # Check for movement to transition to DRIVE mode
            if accel_change > 0.3 or gyro_magnitude > 15.0:  # Thresholds for movement detection
                current_state = STATE_DRIVE
                # State change is important, send immediately
                state_update = {
                    "system": {
                        "state": "DRIVE",
                        "event": "movement_detected"
                    }
                }
                send_to_firebase(state_update, is_critical=True)
                send_to_ubidots(state_update, is_critical=True)
                print("Movement detected, switching to DRIVE mode")
                last_movement_time = current_time
                
                # Reset accident buffer when entering DRIVE mode
                accident_buffer.clear()
        
        elif current_state == STATE_DRIVE:
            # First check for accident
            if detect_accident(sensor_data):
                handle_accident()
            
            # Check for movement to stay in DRIVE mode
            elif accel_change > 0.3 or gyro_magnitude > 15.0:
                # Reset the timer if movement detected
                last_movement_time = current_time
            
            # If minimal movement for 20 seconds, switch back to PARK mode
            elif current_time - last_movement_time > 20:
                current_state = STATE_PARK
                # State change is important, send immediately
                state_update = {
                    "system": {
                        "state": "PARK",
                        "event": "no_movement"
                    }
                }
                send_to_firebase(state_update, is_critical=True)
                send_to_ubidots(state_update, is_critical=True)
                print("No movement detected for 20s, switching to PARK mode")
        
        elif current_state == STATE_ACCIDENT:
            # Monitor after accident - if minimal movement for 20 seconds, return to PARK mode
            if accel_change > 0.3 or gyro_magnitude > 15.0:
                # Reset the timer if movement detected
                last_movement_time = current_time
            elif current_time - last_movement_time > 20:
                current_state = STATE_PARK
                # State change is important, send immediately
                state_update = {
                    "system": {
                        "state": "PARK",
                        "event": "recovery_from_accident"
                    }
                }
                send_to_firebase(state_update, is_critical=True)
                send_to_ubidots(state_update, is_critical=True)
                print("No movement after accident for 20s, switching to PARK mode")
    
    except Exception as e:
        print(f"Error in send_sensor_data: {e}")

# Safe timer initialization
def setup_timers():
    global data_timer, gps_timer
    
    # Initialize variables to None first
    data_timer = None
    gps_timer = None
    
    try:
        # Setup timer for sending sensor data (5000ms = 0.2Hz)
        data_timer = Timer(0)
        data_timer.init(period=5000, mode=Timer.PERIODIC, callback=send_sensor_data)
        print("Sensor data timer initialized (0.2Hz)")
        
        # Setup timer for GPS reading (10000ms = 0.1Hz)
        gps_timer = Timer(1)
        gps_timer.init(period=10000, mode=Timer.PERIODIC, callback=read_gps)
        print("GPS read timer initialized (0.1Hz)")
        
        return True
    except Exception as e:
        print(f"Error setting up timers: {e}")
        
        # Clean up any timers that might have been created
        try:
            if data_timer is not None:
                data_timer.deinit()
                data_timer = None
        except:
            pass
            
        try:
            if gps_timer is not None:
                gps_timer.deinit()
                gps_timer = None
        except:
            pass
            
        return False

# Ping Ubidots to keep connection alive
def ping_ubidots(timer=None):
    global ubidots_client
    try:
        if ubidots_client is not None:
            ubidots_client.ping()
    except Exception as e:
        print(f"Error pinging Ubidots: {e}")
        # Try to reconnect
        try:
            init_ubidots()
        except:
            pass

# Safe cleanup function
def safe_cleanup():
    global data_timer, gps_timer, ubidots_client
    
    print("Performing system cleanup...")
    
    # Clean up timers
    try:
        if data_timer is not None:
            data_timer.deinit()
            print("Data timer deinitialized")
    except Exception as e:
        print(f"Error cleaning up data timer: {e}")
    
    try:
        if gps_timer is not None:
            gps_timer.deinit()
            print("GPS timer deinitialized")
    except Exception as e:
        print(f"Error cleaning up GPS timer: {e}")
    
    # Disconnect from Ubidots
    try:
        if ubidots_client is not None:
            ubidots_client.disconnect()
            print("Ubidots client disconnected")
    except Exception as e:
        print(f"Error disconnecting from Ubidots: {e}")
    
    print("System cleanup complete")

# Main function
def main():
    global current_state, accident_buffer, last_data_send
    
    print("=== ESP32 Accident Detection System with Firebase and Ubidots ===")
    
    # Initialize accident buffer and timers
    accident_buffer = []
    last_data_send = 0
    
    # Initialize hardware
    if not initialize_hardware():
        print("Critical error: Hardware initialization failed!")
        return
    
    # Initialize components
    if not init_mpu6050():
        print("Failed to initialize MPU6050, retrying...")
        time.sleep(1)
        if not init_mpu6050():
            print("Critical error: MPU6050 initialization failed!")
            return
    
    # Connect to Wi-Fi
    if not connect_wifi():
        print("Critical error: Wi-Fi connection failed!")
        return
    
    # Initialize Firebase
    if not init_firebase():
        print("Critical error: Firebase initialization failed!")
        return
    
    # Initialize Ubidots
    if not init_ubidots():
        print("Warning: Ubidots initialization failed, will retry during operation")
    
    # Test connection to Firebase
    print("Testing connection to Firebase...")
    startup_data = {
        "system": {
            "event": "startup",
            "state": "PARK"
        }
    }
    if not send_to_firebase(startup_data):
        print("Warning: Initial connection to Firebase failed, will retry during operation")
    else:
        print("Successfully connected to Firebase!")
    
    # Test connection to Ubidots
    print("Testing connection to Ubidots...")
    if not send_to_ubidots(startup_data):
        print("Warning: Initial connection to Ubidots failed, will retry during operation")
    else:
        print("Successfully connected to Ubidots!")
    
    # Set system state to PARK
    current_state = STATE_PARK
    
    # Setup timers for periodic checks
    if not setup_timers():
        print("Warning: Timer setup failed, continuing with limited functionality")
    
    # Setup timer for Ubidots ping (keep connection alive)
    ubidots_ping_timer = Timer(2)
    ubidots_ping_timer.init(period=60000, mode=Timer.PERIODIC, callback=ping_ubidots)  # Every 60 seconds
    
    print("System initialized and running in PARK mode")
    
    try:
        # Main loop - keep the system running
        while True:
            gc.collect()
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("Program terminated by user")
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        # Clean up
        safe_cleanup()

# Start the program
if __name__ == "__main__":
    main()

