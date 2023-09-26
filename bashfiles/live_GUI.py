#!/usr/bin/env python3
"""A simple example of a live GUI using CTk"""
import customtkinter
customtkinter.set_appearance_mode('System')
customtkinter.set_default_color_theme('blue')

app = customtkinter.CTk()
app.geometry('640x480')

def button_function():
    """a CTk button function"""
    print('Button pressed!')
def button_exit():
    """a CTk button function"""
    app.destroy()

btn_press_me = customtkinter.CTkButton(master=app, text='Press me!', command=button_function)
btn_exit = customtkinter.CTkButton(master=app, text='Exit', command=button_exit)

btn_press_me.place(relx=0.5, rely=0.5, anchor='center')
btn_exit.place(relx=0.8, rely=0.9, anchor='center')



app.mainloop()
