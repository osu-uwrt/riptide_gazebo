from tkinter import *
from tkinter import colorchooser
from PIL import Image
import os

imgs_path = os.getcwd()+'/imgs'
fog_color = 'none selected'
spec_color = 'none selected'
diff_color = 'none selected'
app_height = 450

#callbacks
def generate_callback():
    print('Generate Runs!')

def getColor():
    color = colorchooser.askcolor()
    print(color)
    return color

# create the application
class Application(Frame):
    def __init__(self, master=None, title='Simulator'):
        super().__init__(master)
        self.master = master

        #Environment Pane (Left-most Pane)
        frame1 = Frame(master=master, width=150, height=app_height)
        frame1.pack(side=LEFT, expand=True, fill=BOTH, pady=3, padx=3) 

        #Sun   
        frame1A = Frame(master=frame1, relief=RAISED)
        frame1A.grid(row=1)
        sunLabel = Label(master=frame1, text='Sun')
        sunLabel.grid(row=0, column=0)
        directionLabel = Label(master=frame1A, text='Direction:')
        directionLabel.grid(row=1, column=0)
        directionScale = Scale(master=frame1A, from_=-180, to=180, orient=HORIZONTAL)
        directionScale.grid(row=1, column=1)
        specular = Button(master=frame1A, text='Specular', command=getColor)
        specular.grid(row=2,column=0)
        specularLabel = Label(master=frame1A, textvariable=spec_color)
        specularLabel.grid(row=2,column=1)
        diffuse = Button(master=frame1A, text='Diffuse', command=getColor)
        diffuse.grid(row=3,column=0)
        diffuseLabel = Label(master=frame1A, textvariable=diff_color)
        diffuseLabel.grid(row=3,column=1)

        #Fog
        frame1B = Frame(master=frame1, relief=RAISED)
        frame1B.grid(row=3)
        fogLabel = Label(master=frame1, text='Fog')
        fogLabel.grid(row=2)
        #fog color
        frame1B1 = Frame(master=frame1B, relief=RAISED)
        frame1B1.grid(row=0)
        color = Button(master=frame1B1, text='Color', command=getColor)
        color.grid(row=0, column=0)
        colorLabel = Label(master=frame1B1, textvariable=fog_color)
        colorLabel.grid(row=1, column=0)
        #fog std
        colorstdScale = Scale(master=frame1B, from_=100, to=0, length=50, orient=VERTICAL)
        colorstdScale.grid(row=0, column=1)       
        #fog thickness
        thicknessLabel = Label(master=frame1B, text='Thickness:')
        thicknessLabel.grid(row=2, column=0)
        Scale(master=frame1B, from_=0, to=100, orient=HORIZONTAL).grid(row=2, column=1)

        #Number Of Runs (Top of Environment Pane)
        frame1C = Frame(master=frame1, relief=RAISED)
        frame1C.grid()
        numberRunsLabel = Label(master=frame1C, text='Total Runs: ')
        numberRunsLabel.grid()
        #numberRuns = Entry(master=frame1C, width=4)
        numberRuns = Spinbox(frame1C, from_=1, to=1000, width=5)
        numberRuns.grid()

        #Generate Runs Button
        generateButton = Button(master=frame1, text="Generate", fg='black', command = generate_callback)
        generateButton.grid()

        #Transdec Air Map (Middle Pane)
        frame2 = Frame(master=master, width=600, height=app_height, bg="black")
        canvas = Canvas(frame2, width=500, height=app_height, bg="black")
        canvas.grid()
        transdec_png = PhotoImage(file=imgs_path+'/transdec_air.PNG')
        root.transdec_png = transdec_png
        transdec_img = canvas.create_image((0,75), image=transdec_png, anchor=NW)
        frame2.pack(fill=BOTH, side=LEFT, expand=True)        
        
        #Props Pane (Right-most Pane)
        frame3 = Frame(master=master, width=150, height=300)
        frame3.pack(fill=BOTH, side=LEFT, expand=True)

        #Library of Props
        libraryPane = Frame(master=frame3, bg='white')
        libraryPane.grid()
        propsLabel = Label(master=libraryPane, text='Props')
        propsLabel.grid()
        #gate
        gate_frame = Frame(master=libraryPane)
        gate_frame.grid(row=1, column=0)
        gate_png = PhotoImage(file=imgs_path+'/gate.png')
        root.gate_png = gate_png
        libraryCanvas = Canvas(master=gate_frame, width=25, height=25, relief=RAISED)
        libraryCanvas.grid()
        gate_img = libraryCanvas.create_image((0,0), image=gate_png, anchor=NW)
        gateLabel = Label(master=gate_frame, text='gate')
        gateLabel.grid(row=1)
        #crucifix
        crucifix_frame = Frame(master=libraryPane)
        crucifix_frame.grid(row=1, column=1)
        crucifix_png = PhotoImage(file=imgs_path+'/crucifix.png')
        root.crucifix_png = crucifix_png
        libraryCanvas = Canvas(master=crucifix_frame, width=25, height=25, relief=RAISED)
        libraryCanvas.grid()
        crucifix_img = libraryCanvas.create_image((0,0), image=crucifix_png, anchor=NW)
        crucifixLabel = Label(master=crucifix_frame, text='crucifix')
        crucifixLabel.grid(row=1)
        #Prop Properties


#define frame classes

# start the program
root=Tk()
home = Application(master=root)
home.mainloop()