#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Tkinter as tk
import ttk
from PIL import Image, ImageTk


# Clase que inicializa la pestaña 1 -> Información
class InformationFrame(ttk.Frame, object):

    def __init__(self, *args, **kwargs):
        super(InformationFrame, self).__init__(*args, **kwargs)

# Clase que inicializa la pestaña 2 -> De momento, nada
class SecondFrame(ttk.Frame, object):

     def __init__(self, *args, **kwargs):
        super(SecondFrame, self).__init__(*args, **kwargs)

# Clase que inicializa la aplicación gráfica
class Application(ttk.Frame, object):

    # Función de inicialización de la ventana gráfica
    def __init__(self, root):

        # Inicialización de la ventana
        super(Application, self).__init__()
        self.window = root
        self.window.title('HMI Eurobot')
        self.window.geometry("1280x720")
        self.window.resizable(width = "false", height = "false")

        # Inicialización del canvas para dibujar o meter imágenes encima
        self.canvas = tk.Canvas(self.window, width = 1280, height = 720, bg = "#0354a7")
        self.canvas.place(x = 0, y = 0)

        # Inicialización del sistema de pestañas
        self.notebook = ttk.Notebook(self.window, width = 1278, height = 647)
        self.notebook.place(x = 0, y = 50)
        
        self.infoFrame = InformationFrame(self.notebook)
        self.notebook.add(self.infoFrame, text = "Información general", padding = 14)
        
        self.secFrame = SecondFrame(self.notebook)
        self.notebook.add(self.secFrame, text = "Pestaña2", padding = 14)

        # Elementos de la ventana principal
        self.textTitle = tk.Label(self.window, text = "HMI Eurobot", fg = "light blue", bg = "#0354a7", font = ('Helvetica', 32, 'bold italic'))
        self.textTitle.place(x = 560, y = 5)

        self.path = "/home/santi/Ros_ws/src/Pantalla/src/"
        self.imgUni = ImageTk.PhotoImage(Image.open(self.path + "logo-UAH-blue.png"))
        self.canvas.create_image(1200, 25, image = self.imgUni)       
        
