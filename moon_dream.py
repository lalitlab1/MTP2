import torch
from transformers import AutoModelForCausalLM, AutoTokenizer
from PIL import Image
import requests
from io import BytesIO

# Download and save the model
def download_and_save_model(model_name, save_path):
    # Load model and tokenizer
    model = AutoModelForCausalLM.from_pretrained(
        model_name,
        revision="2025-01-09",
        trust_remote_code=True,
    )
    tokenizer = AutoTokenizer.from_pretrained(model_name, trust_remote_code=True)
    
    # Save the model and tokenizer
    model.save_pretrained(save_path)
    tokenizer.save_pretrained(save_path)
    print(f"Model and tokenizer saved to {save_path}")

    return model, tokenizer


# Load image from URL (or local file)
def load_image(image_path):
    if image_path.startswith("http"):
        response = requests.get(image_path)
        image = Image.open(BytesIO(response.content))
    else:
        image = Image.open(image_path)
    return image


# Use the model for captioning and queries
def use_model_for_queries(model, image, query):
    print("\nCaptioning the image:")
    
    # Short captioning
    print("Short caption:")
    caption_short = model.caption(image, length="short")["caption"]
    print(caption_short)
    
    # Normal captioning
    print("\nNormal caption:")
    for t in model.caption(image, length="normal", stream=True)["caption"]:
        print(t, end="", flush=True)
    print()
    caption_normal = model.caption(image, length="normal")["caption"]
    print(caption_normal)
    
    # Visual Querying
    print("\nVisual query: 'How many people are in the image?'")
    query_result = model.query(image, query)["answer"]
    print(f"Answer: {query_result}")
    
    # Object Detection: Detecting faces
    print("\nObject detection: 'face'")
    objects = model.detect(image, "face")["objects"]
    print(f"Found {len(objects)} face(s)")
    
    # Pointing: Detecting people
    print("\nPointing: 'person'")
    points = model.point(image, "person")["points"]
    print(f"Found {len(points)} person(s)")


# Main function to download model, load image, and run queries
def main():
    # Download and save the model
    model_name = "vikhyatk/moondream2"
    save_path = "./moondream2_model"
    model, tokenizer = download_and_save_model(model_name, save_path)
    
    # Load image
    image_path = "truck.jpg"  # Can be a URL or local path
    image = load_image(image_path)

    # Use the model for queries
    query = "Describe the image"
    use_model_for_queries(model, image, query)


if __name__ == "__main__":
    main()
