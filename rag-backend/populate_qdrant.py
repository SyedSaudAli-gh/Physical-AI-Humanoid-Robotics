"""
Script to populate Qdrant collection with sample data for the Physical AI & Humanoid Robotics textbook
"""
import os
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import logging
from mock_embedding import mock_embed_text

# Load environment variables
load_dotenv()

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configure Google Generative AI
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    timeout=10
)

# Configuration
COLLECTION_NAME = "physical_ai_docs"

def recreate_collection():
    """Recreate the Qdrant collection with correct dimensions"""
    try:
        # Check if collection exists and delete it
        collections = qdrant_client.get_collections()
        collection_names = [c.name for c in collections.collections]
        
        if COLLECTION_NAME in collection_names:
            logger.info(f"Deleting existing collection '{COLLECTION_NAME}'...")
            qdrant_client.delete_collection(COLLECTION_NAME)
            logger.info(f"Collection '{COLLECTION_NAME}' deleted")
        
        logger.info(f"Creating collection '{COLLECTION_NAME}' with 768 dimensions...")
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(
                size=768,  # Google embeddings size for 'models/embedding-001'
                distance=models.Distance.COSINE
            )
        )
        logger.info(f"Collection '{COLLECTION_NAME}' created successfully with correct dimensions")
    except Exception as e:
        logger.error(f"Error creating collection: {e}")
        raise e

def embed_text(text: str) -> list:
    """Convert text to embedding using Google's embedding service or mock if quota exceeded"""
    try:
        result = genai.embed_content(
            model="models/embedding-001",
            content=[text],
            task_type="retrieval_document"  # Specify this is for document retrieval
        )
        return result['embedding'][0]
    except Exception as e:
        logger.warning(f"Error using Google embedding service: {e}. Using mock embedding.")
        return mock_embed_text(text)

def populate_sample_data():
    """Populate the collection with sample data"""
    sample_data = [
        {
            "id": 1,
            "content": "Physical AI refers to the integration of artificial intelligence algorithms with physical systems, enabling robots and other devices to perceive, reason, and act in the real world. This field combines machine learning, robotics, and control theory.",
            "source_url": "/docs/intro",
            "page_title": "Introduction to Physical AI"
        },
        {
            "id": 2,
            "content": "Humanoid robotics is a branch of robotics focused on creating robots with human-like form and behavior. These robots typically have a head, torso, two arms, and two legs, and are designed to interact with human environments.",
            "source_url": "/docs/humanoid-robotics",
            "page_title": "Humanoid Robotics Fundamentals"
        },
        {
            "id": 3,
            "content": "The Robot Operating System (ROS) is a flexible framework for writing robot software. It provides services such as hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.",
            "source_url": "/docs/ros-fundamentals",
            "page_title": "ROS 2 Fundamentals"
        },
        {
            "id": 4,
            "content": "Gazebo is a robot simulator that provides realistic 3D environments for simulating robots. It's widely used in the robotics community for testing algorithms, designs, and scenarios.",
            "source_url": "/docs/simulation",
            "page_title": "Gazebo & Unity Simulation"
        },
        {
            "id": 5,
            "content": "NVIDIA Isaac is a robotics platform that provides tools and technologies for developing, simulating, and deploying AI-powered robots. It includes Isaac Sim for simulation and Isaac ROS for perception.",
            "source_url": "/docs/nvidia-isaac",
            "page_title": "NVIDIA Isaac Navigation"
        },
        {
            "id": 6,
            "content": "Vision-Language-Action models integrate visual perception, natural language understanding, and robotic action planning. These multimodal systems enable robots to follow complex instructions and perform tasks based on visual and linguistic inputs.",
            "source_url": "/docs/vision-language-action",
            "page_title": "Vision-Language-Action Models"
        },
        {
            "id": 7,
            "content": "Embodied intelligence refers to the idea that intelligence emerges from the interaction between an agent and its environment. It emphasizes the importance of physical embodiment in developing intelligent behavior.",
            "source_url": "/docs/embodied-intelligence",
            "page_title": "Embodied Intelligence"
        },
        {
            "id": 8,
            "content": "Sensor fusion combines data from multiple sensors to improve the accuracy and reliability of perception systems. Common sensors in humanoid robots include cameras, IMUs, force/torque sensors, and LiDAR.",
            "source_url": "/docs/sensor-fusion",
            "page_title": "Sensor Fusion in Robotics"
        }
    ]
    
    try:
        logger.info(f"Preparing to add {len(sample_data)} documents to collection '{COLLECTION_NAME}'...")
        
        # Prepare points for insertion
        points = []
        for item in sample_data:
            vector = embed_text(item["content"])
            
            point = models.PointStruct(
                id=item["id"],
                vector=vector,
                payload={
                    "content": item["content"],
                    "source_url": item["source_url"],
                    "page_title": item["page_title"]
                }
            )
            points.append(point)
        
        # Insert points into collection
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
        
        logger.info(f"Successfully added {len(sample_data)} documents to collection '{COLLECTION_NAME}'")
        
    except Exception as e:
        logger.error(f"Error populating sample data: {e}")
        raise e

if __name__ == "__main__":
    logger.info("Starting Qdrant collection recreation and population...")
    
    recreate_collection()
    populate_sample_data()
    
    logger.info("Qdrant collection population completed successfully!")
    
    # Test a simple search to verify everything is working
    try:
        from retrieval_service import retrieve
        
        logger.info("Testing retrieval with a sample query...")
        result = retrieve("What is Physical AI?", top_k=3)
        
        logger.info(f"Retrieved {result.total_results} results for test query")
        for chunk in result.retrieved_chunks:
            logger.info(f"  - {chunk.page_title}: {chunk.content[:100]}...")
            
    except Exception as e:
        logger.error(f"Error during test retrieval: {e}")