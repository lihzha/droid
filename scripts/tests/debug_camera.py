import cv2

def start_recording(filename, frame_width, frame_height, fps=30.0):
    """Initialize the video writer for recording."""
    if filename.endswith(".mp4"):
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # Codec for .mp4
    elif filename.endswith(".avi"):
        fourcc = cv2.VideoWriter_fourcc(*"XVID")  # Codec for .avi
    else:
        raise ValueError("Unsupported file format. Use .mp4 or .avi")
    
    # Initialize VideoWriter object
    video_writer = cv2.VideoWriter(filename, fourcc, fps, (frame_width, frame_height))
    
    # Check if VideoWriter is opened
    if not video_writer.isOpened():
        print("Error: Could not open video writer.")
        return None
    return video_writer

def stop_recording(video_writer):
    """Stop the recording and save the video."""
    if video_writer is not None:
        video_writer.release()
        print("Recording stopped and video saved.")
    else:
        print("No active recording found.")

# Open a connection to the webcam (change camera index as needed)
cap = cv2.VideoCapture(2)

# Check if the webcam is opened correctly
if not cap.isOpened():
    print("Error: Could not open the webcam.")
    exit()

# Set the camera resolution
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Get actual frame width and height
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(frame_width, frame_height)

# Initialize video recording
filename = "webcam_recording.mp4"  # Change filename if needed
video_writer = start_recording(filename, frame_width, frame_height)

if video_writer is None:
    print("Error: VideoWriter failed to initialize.")
    cap.release()
    cv2.destroyAllWindows()
    exit()

# Capture and display the webcam feed
while True:
    ret, frame = cap.read()  # Read a frame from the webcam

    if not ret:
        print("Failed to capture image.")
        break

    # Display the frame
    cv2.imshow('Webcam Feed', frame)

    # Write the frame to the video file if recording is active
    if video_writer is not None:
        # Ensure frame size matches the video writer's expected size
        resized_frame = cv2.resize(frame, (frame_width, frame_height))
        video_writer.write(resized_frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop recording and release resources
stop_recording(video_writer)
cap.release()
cv2.destroyAllWindows()
