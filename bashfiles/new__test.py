# create a class called Person
# create init method
# 2 attributes (name, and birthdate)
# create an object from the Person class


from PIL import ImageTk, Image
import datetime
import tkinter as tk
import os

# create frame
window = tk.Tk()

# create frame geometry
window.geometry("400x400")

# set title of frame
window.title("Age Calculator App")



# adding labels
name_label = tk.Label(text="Name")
name_label.grid(column=0, row=0)

year_label = tk.Label(text="Year")
year_label.grid(column = 0, row = 1)

month_label = tk.Label(text="Month")
month_label.grid(column = 0, row = 2)

day_label = tk.Label(text="Day")
day_label.grid(column = 0, row = 3)


# adding entries
name_entry = tk.Entry()
name_entry.grid(column=1, row=0)

year_entry = tk.Entry()
year_entry.grid(column=1, row=1)

month_entry = tk.Entry()
month_entry.grid(column=1, row=2)

day_entry = tk.Entry()
day_entry.grid(column=1, row=3)


def calculate_age():
    year = int(year_entry.get())
    month = int(month_entry.get())
    day = int(day_entry.get())
    name = name_entry.get()
    person = Person(name, datetime.date(year, month, day))
    text_answer = tk.Text(master=window, wrap=tk.WORD, height=20, width=30)
    text_answer.grid(column= 1, row= 5)
    answer = "{name} is {age} years old!".format(name=person.name, age=person.age())
    is_old_answer = " Wow you are getting old aren't you?"
    text_answer.insert(tk.END, answer)
    if person.age() >= 50:
        text_answer.insert(tk.END, is_old_answer)


calculate_button = tk.Button(text="Calculate Age!", command=calculate_age)
calculate_button.grid(column=1, row=4)


class Person:
    def __init__(self, name, birthdate):
        self.name = name
        self.birthdate = birthdate

    def age(self):
        today = datetime.date.today()
        age = today.year - self.birthdate.year
        return age
# get present directory of the File
path = '/home/ammar/ros_ws/bashfiles/test.png'
print(path)

image = Image.open(path)
# image.thumbnail((100, 100), Image.ANTIALIAS)
photo = tk.PhotoImage(file=image)
label_image = tk.Label(image=image)
label_image.grid(column=1, row=0)


window.mainloop()