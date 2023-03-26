from flask import Flask, render_template, request
import tkinter as tk

app = Flask(__name__)

class SaveCredentialsGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Save Credentials")

        # Create and position the username and password entry widgets
        tk.Label(master, text="Username:").grid(row=0)
        self.username_entry = tk.Entry(master)
        self.username_entry.grid(row=0, column=1)

        tk.Label(master, text="Password:").grid(row=1)
        self.password_entry = tk.Entry(master, show="*")
        self.password_entry.grid(row=1, column=1)

        # Create and position the save button
        self.save_button = tk.Button(master, text="Save", command=self.save_credentials)
        self.save_button.grid(row=2, columnspan=2)

    def save_credentials(self):
        # Get the entered username and password
        username = self.username_entry.get()
        password = self.password_entry.get()

        # Write the credentials to a file
        with open("credentials.txt", "a") as f:
            f.write(f"Username: {username}\n")
            f.write(f"Password: {password}\n\n")

        # Clear the entry widgets
        self.username_entry.delete(0, tk.END)
        self.password_entry.delete(0, tk.END)

@app.route("/", methods=["GET", "POST"])
def save_credentials():
    if request.method == "POST":
        # Create the tkinter GUI and save the credentials
        root = tk.Tk()
        app = SaveCredentialsGUI(root)
        root.mainloop()
        return "Credentials saved!"
    else:
        # Render the HTML template
        return render_template("save_credentials.html")

if __name__ == "__main__":
    app.run()
