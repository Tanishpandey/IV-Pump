import tkinter as tk
from PIL import Image, ImageTk

# Create Tkinter window
root = tk.Tk()
root.title("Click to get coordinates")

# Load the image
image_path = "Alaris-8015-4.png"  # or full path
image = Image.open(image_path)
photo = ImageTk.PhotoImage(image)

# Create a label widget with the image
label = tk.Label(root, image=photo)
label.pack()

# Function to handle mouse clicks
def print_coords(event):
    print(f"Clicked at: x={event.x}, y={event.y}")

# Bind left mouse click to the function
label.bind("<Button-1>", print_coords)

root.mainloop()
