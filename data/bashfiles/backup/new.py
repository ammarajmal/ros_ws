#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import customtkinter


class App(customtkinter.CTk):
    """_summary_

    Args:
        customtkinter (_type_): _description_
    """
    def __init__(self):
        super().__init__()

        # create a label
        self.label1 = customtkinter.CTkLabel(self, text="Label 1")
        self.label1.grid(row=0, column=0)

        # create a button
        self.button1 = customtkinter.CTkButton(self, text="Button 1", command=self.button1_click)
        self.button1.grid(row=1, column=0)

        # create a label
        self.label2 = customtkinter.CTkLabel(self, text="Label 2")
        self.label2.grid(row=0, column=1)

        # create a button
        self.button2 = customtkinter.CTkButton(self, text="Button 2", command=self.button2_click)
        self.button2.grid(row=1, column=1)

    def button1_click(self):
        """_summary_
        """
        # handle button 1 click event
        self.label1.config(text="Button 1 clicked!")

    def button2_click(self):
        """_summary_
        """
        # handle button 2 click event
        self.label2.config(text="Button 2 clicked!")
if __name__ == "__main__":
    app = App()
    app.mainloop()