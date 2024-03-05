import tkinter as tk
import webbrowser

def open_url():
    url = "http://localhost:3000/d/d6afadb7-6453-432c-870f-9758a492d2e7/all-nuc-displaement?orgId=1"
    webbrowser.open(url)

# Create a Tkinter window
window = tk.Tk()
window.title("Open URL")

# Create a button to open the URL
button = tk.Button(window, text="Open URL", command=open_url)
button.pack()

# Start the Tkinter main loop
window.mainloop()
