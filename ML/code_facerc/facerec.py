import cv2
import numpy as np
import face_recognition
import os
import time
import threading
import queue
import paho.mqtt.client as mqtt
from datetime import datetime
import base64
from flask import Flask, request, jsonify, Response
import shutil




UPLOAD_FOLDER = 'uploads'
LATEST_IMAGE = 'latest.jpg'
MAX_IMAGES = 30  

if not os.path.exists(UPLOAD_FOLDER):
    os.makedirs(UPLOAD_FOLDER)

class FaceRecognitionSystem:
    def __init__(self, known_faces_dir="known_faces", 
             detection_interval=1.0,
             mqtt_broker="192.168.18.38",
             mqtt_port=1883,
             mqtt_topic="security/face_detection",
             mqtt_auth_topic="esp32/auth/user_detection",  
             mqtt_username="UNI022",
             mqtt_password="022",
             tolerance=0.6):
        # Base configuration
        self.known_faces_dir = known_faces_dir
        self.known_face_encodings = []
        self.known_face_names = []
        self.detection_interval = detection_interval
        self.tolerance = tolerance
        
        # MQTT configuration
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.mqtt_topic = mqtt_topic
        self.mqtt_auth_topic = mqtt_auth_topic 
        self.mqtt_username = mqtt_username
        self.mqtt_password = mqtt_password
        self.mqtt_client = None

        self.frame_queue = queue.Queue(maxsize=2) 
        self.is_running = False
        self.last_detection_time = 0
        self.lock = threading.Lock()

        self.current_frame = None
        self.latest_processed_frame = None
        
        # Load known faces
        self.load_known_faces()
        self.init_mqtt()
    
    def init_mqtt(self):
        """Initialize MQTT connection"""
        try:
            self.mqtt_client = mqtt.Client()
            
            # Add username and password
            self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
            
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            print(f"MQTT client connected to {self.mqtt_broker}:{self.mqtt_port}")
        except Exception as e:
            print(f"MQTT connection error: {e}")
            self.mqtt_client = None
    
    def publish_mqtt(self, message, auth_message=None):
        """Send message to MQTT topic"""
        if self.mqtt_client:
            try:
                # Send to regular topic
                result = self.mqtt_client.publish(self.mqtt_topic, message)
                if result.rc == mqtt.MQTT_ERR_SUCCESS:
                    print(f"MQTT message sent: {message}")
                else:
                    print(f"MQTT error: {result.rc}")
                
                # Send to authentication topic if a known user is detected
                if auth_message:
                    
                    json_message = '{"user_detected": true}'
                    auth_result = self.mqtt_client.publish(self.mqtt_auth_topic, json_message)
                    if auth_result.rc == mqtt.MQTT_ERR_SUCCESS:
                        print(f"Auth MQTT message sent to {self.mqtt_auth_topic}: {json_message}")
                    else:
                        print(f"Auth MQTT error: {auth_result.rc}")
                        
            except Exception as e:
                print(f"Error sending MQTT: {e}")
    
    def load_known_faces(self):
        """Load known faces from directory"""
        start_time = time.time()
        print("Loading face database...")

        if not os.path.exists(self.known_faces_dir):
            os.makedirs(self.known_faces_dir)
            print(f"Directory {self.known_faces_dir} created. Please add known face images.")
            return
        
        # Load each image file in the directory
        count = 0
        for filename in os.listdir(self.known_faces_dir):
            if filename.endswith(('.png', '.jpg', '.jpeg')):
                
                name = os.path.splitext(filename)[0]
                filepath = os.path.join(self.known_faces_dir, filename)
                
                try:
                    
                    image = face_recognition.load_image_file(filepath)
                    
                    face_locations = face_recognition.face_locations(image, model="hog")
                    
                    if face_locations:
            
                        face_encoding = face_recognition.face_encodings(image, face_locations)[0]
                        
                        self.known_face_encodings.append(face_encoding)
                        self.known_face_names.append(name)
                        count += 1
                        print(f"Face {name} successfully loaded")
                    else:
                        print(f"No face detected in {filename}")
                except Exception as e:
                    print(f"Error loading {filename}: {e}")
        
        elapsed_time = time.time() - start_time
        print(f"Successfully loaded {count} faces in {elapsed_time:.2f} seconds")
    
    def add_new_face(self, image_path, name):
        """Add new face to database"""
        try:
            # Load image
            image = face_recognition.load_image_file(image_path)
            
            face_locations = face_recognition.face_locations(image, model="hog")
            
            if face_locations:
                
                face_encoding = face_recognition.face_encodings(image, face_locations)[0]
                
                
                self.known_face_encodings.append(face_encoding)
                self.known_face_names.append(name)
                
                if not os.path.exists(self.known_faces_dir):
                    os.makedirs(self.known_faces_dir)
                
                extension = os.path.splitext(image_path)[1]
                new_path = os.path.join(self.known_faces_dir, f"{name}{extension}")
                
                if os.path.exists(new_path):
                    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
                    new_path = os.path.join(self.known_faces_dir, f"{name}_{timestamp}{extension}")
                
                import shutil
                shutil.copy2(image_path, new_path)
                
                print(f"Face {name} successfully added")
                return True
            else:
                print(f"No face detected in {image_path}")
                return False
        except Exception as e:
            print(f"Error adding new face: {e}")
            return False
    
    def process_frame(self, frame):
        """Process a single frame for face recognition"""
        try:

            small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
            
            rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
            
            with self.lock:
                self.current_frame = frame.copy()
            
            face_locations = face_recognition.face_locations(rgb_small_frame, model="hog")

            processed_frame = frame.copy()
            

            if not face_locations:
                print("No face detected")
                self.publish_mqtt("no_face_detected")
                return processed_frame, "no face"
            
            face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
            
            detection_result = None
            
            for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):

                top *= 2
                right *= 2
                bottom *= 2
                left *= 2
                
                # Compare with known faces
                if len(self.known_face_encodings) > 0:
                    matches = face_recognition.compare_faces(
                        self.known_face_encodings, 
                        face_encoding,
                        tolerance=self.tolerance
                    )
                    
                    # If match found
                    if True in matches:
                        
                        match_index = matches.index(True)
                        name = self.known_face_names[match_index]
                        
                        cv2.rectangle(processed_frame, (left, top), (right, bottom), (0, 255, 0), 2)
                        cv2.rectangle(processed_frame, (left, bottom - 35), (right, bottom), (0, 255, 0), cv2.FILLED)
                        cv2.putText(processed_frame, name, (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 1.0, (255, 255, 255), 1)
                        
                        print(f"Face detected: {name}")
                       
                        self.publish_mqtt(f"face_{name}_detected", "user_detected")
                        detection_result = name
                    else:
                        # If no match = intruder
                        cv2.rectangle(processed_frame, (left, top), (right, bottom), (0, 0, 255), 2)
                        cv2.rectangle(processed_frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                        cv2.putText(processed_frame, "Unknown", (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 1.0, (255, 255, 255), 1)
                        
                        print("Face detected: intruder")
                        self.publish_mqtt("face_intruder_detected")
                        detection_result = "intruder"
                else:
                    # If no known faces in database
                    cv2.rectangle(processed_frame, (left, top), (right, bottom), (0, 0, 255), 2)
                    cv2.rectangle(processed_frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                    cv2.putText(processed_frame, "Unknown", (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 1.0, (255, 255, 255), 1)
                    
                    print("Face detected: intruder (empty database)")
                    self.publish_mqtt("face_intruder_detected")
                    detection_result = "intruder"
            
            
            with self.lock:
                self.latest_processed_frame = processed_frame.copy()
            
            return processed_frame, detection_result
            
        except Exception as e:
            print(f"Error processing frame: {e}")
            return frame, "error"
    
    def process_rest_image(self, image_data):
        """Process image received from REST API"""
        try:
            
            nparr = np.frombuffer(image_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            
            current_time = time.time()
            if current_time - self.last_detection_time >= self.detection_interval:
                self.last_detection_time = current_time
                processed_frame, result = self.process_frame(frame)
                return True, result
            else:
                return False, "skipped"
                
        except Exception as e:
            print(f"Error processing REST image: {e}")
            return False, f"error: {str(e)}"

def limit_images():
    """Keep the number of images in the uploads folder limited to MAX_IMAGES"""
    image_files = [f for f in os.listdir(UPLOAD_FOLDER) if f.endswith('.jpg') and f != LATEST_IMAGE]
    image_files.sort()  
    
    # Delete the oldest files if the limit is exceeded
    if len(image_files) > MAX_IMAGES:
        for file_to_delete in image_files[:len(image_files) - MAX_IMAGES]:
            os.remove(os.path.join(UPLOAD_FOLDER, file_to_delete))
            print(f"Deleted old image: {file_to_delete}")


face_system = FaceRecognitionSystem(
    known_faces_dir="known_faces",
    detection_interval=1.0,
    mqtt_broker="192.168.18.38",
    mqtt_port=1883,
    mqtt_topic="security/face_detection",
    mqtt_auth_topic="esp32/auth/user_detection", 
    mqtt_username="UNI022",
    mqtt_password="022",
    tolerance=0.6
)

# Route for receiving images
@app.route('/upload', methods=['POST'])
def upload_image():
    try:
      
        data = request.get_json()
        
        if not data or 'image' not in data:
            return jsonify({'error': 'No image data received'}), 400
        
   
        base64_image = data['image']
        
       
        image_data = base64.b64decode(base64_image)
        
       
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"{timestamp}.jpg"
        filepath = os.path.join(UPLOAD_FOLDER, filename)
        
        with open(filepath, 'wb') as f:
            f.write(image_data)
        
      
        latest_path = os.path.join(UPLOAD_FOLDER, LATEST_IMAGE)
        with open(latest_path, 'wb') as f:
            f.write(image_data)
            
        print(f"Image saved as {filepath}")
        
       
        limit_images()
        
        
        processed, result = face_system.process_rest_image(image_data)
        
        response_data = {
            'success': True,
            'message': 'Image received and saved',
            'filename': filename,
            'processed': processed,
            'result': result
        }
        
        return jsonify(response_data)
        
    except Exception as e:
        print(f"Error processing image: {e}")
        return jsonify({'error': str(e)}), 500

def main():
    print("Starting integrated Flask server with face recognition...")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

if __name__ == "__main__":
    main()