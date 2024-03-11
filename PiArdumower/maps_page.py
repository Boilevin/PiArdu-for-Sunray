
def onMap0click(event):
        if event.button is MouseButton.LEFT:
            print("left")
        if event.button is MouseButton.RIGHT:
            print("right")
    
        if event.button is MouseButton.FORWARD:
            print("roll up")
        if event.button is MouseButton.BACK:
            print("roll rev")
        
        if (event.dblclick):
            print(f'data coords {event.xdata} {event.ydata}')
        else:
            print("simple")



    