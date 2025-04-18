from flask import Flask, request, jsonify
import base64
import numpy as np
import cv2
import dlib
import time
import paho.mqtt.client as mqtt
from scipy.spatial import distance as dist
from imutils import face_utils
import imutils
import threading
import os
import json

app = Flask(__name__)

# MQTT Configuration
MQTT_BROKER = "192.168.18.38"  
MQTT_PORT = 1883
MQTT_TOPIC = "driver/status"
MQTT_USERNAME = "UNI022"  
MQTT_PASSWORD = "022"  

# Drowsiness Detection Parameters
EYE_AR_THRESH = 0.3
EYE_AR_CONSEC_FRAMES = 10  
YAWN_THRESH = 20

# Initialize face detector and shape predictor
print("-> Loading the predictor and detector...")
detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
predictor = dlib.shape_predictor('shape_predictor_68_face_landmarks.dat')

client = mqtt.Client()
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

def connect_mqtt():
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        print(f"Connected to MQTT Broker: {MQTT_BROKER}")
    except Exception as e:
        print(f"Failed to connect to MQTT broker: {e}")

# Eye aspect ratio calculation
def eye_aspect_ratio(eye):
    A = dist.euclidean(eye[1], eye[5])
    B = dist.euclidean(eye[2], eye[4])

    C = dist.euclidean(eye[0], eye[3])
    ear = (A + B) / (2.0 * C)
    return ear

def final_ear(shape):

    (lStart, lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
    (rStart, rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]

    leftEye = shape[lStart:lEnd]
    rightEye = shape[rStart:rEnd]

    leftEAR = eye_aspect_ratio(leftEye)
    rightEAR = eye_aspect_ratio(rightEye)

    ear = (leftEAR + rightEAR) / 2.0
    return (ear, leftEye, rightEye)

def lip_distance(shape):
    top_lip = shape[50:53]
    top_lip = np.concatenate((top_lip, shape[61:64]))

    low_lip = shape[56:59]
    low_lip = np.concatenate((low_lip, shape[65:68]))

    top_mean = np.mean(top_lip, axis=0)
    low_mean = np.mean(low_lip, axis=0)

    distance = abs(top_mean[1] - low_mean[1])
    return distance

frame_counter = 0
status = "normal"
last_status = "normal"
status_changed_time = time.time()

@app.route('/upload', methods=['POST'])
def upload_image():
    global frame_counter, status, last_status, status_changed_time
    
    try:
        # Get the base64 image from the request
        content = request.json
        base64_image = content.get('image', '')
        
        if not base64_image:
            return jsonify({"error": "No image data received"}), 400
        
        # Decode the base64 image
        image_data = base64.b64decode(base64_image)
        nparr = np.frombuffer(image_data, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if frame is None:
            return jsonify({"error": "Failed to decode image"}), 400
        
        frame = imutils.resize(frame, width=450)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect faces
        rects = detector.detectMultiScale(gray, scaleFactor=1.1, 
                                        minNeighbors=5, minSize=(30, 30),
                                        flags=cv2.CASCADE_SCALE_IMAGE)
        
        status = "normal" 
        
        # Process each detected face
        for (x, y, w, h) in rects:
            rect = dlib.rectangle(int(x), int(y), int(x + w), int(y + h))
            
            shape = predictor(gray, rect)
            shape = face_utils.shape_to_np(shape)
            
            eye = final_ear(shape)
            ear = eye[0]
            
            distance = lip_distance(shape)
            
          
            leftEye = eye[1]
            rightEye = eye[2]
            leftEyeHull = cv2.convexHull(leftEye)
            rightEyeHull = cv2.convexHull(rightEye)
            cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
            cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)
            
            lip = shape[48:60]
            cv2.drawContours(frame, [lip], -1, (0, 255, 0), 1)
            
           
            if ear < EYE_AR_THRESH:
                frame_counter += 1
                if frame_counter >= EYE_AR_CONSEC_FRAMES:
                    status = "user_sleepy"
                    cv2.putText(frame, "DROWSINESS ALERT!", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                frame_counter = 0
            
            # Check for yawning
            if distance > YAWN_THRESH:
                status = "user_yawn"
                cv2.putText(frame, "Yawn Alert", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.putText(frame, f"EAR: {ear:.2f}", (300, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, f"YAWN: {distance:.2f}", (300, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        
        if len(rects) == 0:
            frame_counter = 0
        
        
        current_time = time.time()
        if status != last_status or (current_time - status_changed_time) > 3:
            payload = json.dumps({"status": status})
            client.publish(MQTT_TOPIC, payload)
            last_status = status
            status_changed_time = current_time
        
      
        
        return jsonify({
            "status": "success", 
            "detection_result": status,
            "ear": float(ear) if 'ear' in locals() else None,
            "yawn_distance": float(distance) if 'distance' in locals() else None
        })
    
    except Exception as e:
        print(f"Error processing image: {str(e)}")
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
   
    connect_mqtt()
    
    app.run(host='0.0.0.0', port=5002, debug=False)