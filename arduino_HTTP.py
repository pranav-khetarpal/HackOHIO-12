# import os
# from dotenv import load_dotenv

# # Load environment variables from .env file
# load_dotenv()

# # Access environment variables
# WIFI_SSID = os.getenv('WIFI_SSID')
# WIFI_PASSWORD = os.getenv('WIFI_PASSWORD')

# # Use these variables in your script
# print(f"WIFI SSID: {WIFI_SSID}")
# print(f"WIFI Password: {WIFI_PASSWORD}")


from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins temporarily
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# @post_router.post("/posts/create", response_model=PostModel)
# async def create_post(post: CreatePostModel, user_id: str = Depends(get_current_user_id)) -> PostModel:
#     """
#     Endpoint to create a new post
#     """
#     post_id = db.collection('posts').document().id
#     post_data = {
#         'id': post_id,
#         'userId': user_id,
#         'content': post.content,
#         'timestamp': datetime.now(pytz.UTC),  # Ensure UTC timezone is used
#         'likes_count': 1,  # Initialize the likes count to 1
#         'comments_count': 0,  # Initialize the comments count to 0
#     }

#     try:
#         # Save the post to Firestore

@app.post("/warning")
async def send_warning(request: Request):
    """
    Endpoint to receive warnings from the ESP32 Dev Module.
    """
    try:
        data = await request.json()  # Parse incoming JSON data
        print(f"Received data: {data}")
        # Process the received data (e.g., log it, or act on it)
        return {"status": "success", "message": "Data received", "data": data}
    except Exception as e:
        print(f"Error receiving data: {e}")
        return {"status": "error", "message": "Failed to process data"}

if __name__ == '__main__':
    import uvicorn

    # Set IP address (replace with your local machine's IP)
    ip_address = '0.0.0.0'  # Listen on all interfaces

    # Start the server
    uvicorn.run(app, host=ip_address, port=8080)
