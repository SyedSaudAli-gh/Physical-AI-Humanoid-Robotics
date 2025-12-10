"""
Backend API for Urdu translation functionality
This is an example of how the /translate-urdu endpoint should be implemented
"""

from flask import Flask, request, jsonify
import openai

app = Flask(__name__)

# Configure OpenAI API
openai.api_key = "your-openai-api-key-here"

@app.route('/translate-urdu', methods=['POST'])
def translate_urdu():
    """
    Translate English text to Urdu using OpenAI GPT-4o-mini
    Preserves technical terms in English
    """
    try:
        data = request.json
        text = data.get('text', '')
        
        if not text:
            return jsonify({'error': 'No text provided'}), 400
        
        # Create the translation prompt with specific instructions for technical terms
        system_prompt = (
            "You are an expert technical translator. "
            "Translate the following robotics/AI textbook content to natural, accurate Urdu. "
            "Preserve all technical terms exactly as in English (e.g., ROS 2, URDF, Gazebo, NVIDIA Isaac, VLA, rclpy) — do not translate them. "
            "Use proper Urdu script and technical vocabulary where possible. "
            "Maintain code blocks, headings, and formatting structure."
        )
        
        response = openai.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": text}
            ],
            temperature=0.3,
            max_tokens=4000
        )
        
        urdu_text = response.choices[0].message.content
        
        return jsonify({"urdu_text": urdu_text})
    
    except Exception as e:
        print(f"Translation error: {str(e)}")
        return jsonify({"error": "Translation failed"}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000, debug=True)