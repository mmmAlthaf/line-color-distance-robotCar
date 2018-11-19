import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import serial

ser = serial.Serial('COM20')
# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []



# This function is called periodically from FuncAnimation
def animate(i, xs, ys):

    num=bytearray(ser.read())
    val=num[0]
    print(val)


    
     # Add x and y to lists
    
    ys.append(val)    
    
    
    

    


    # Draw x and y lists
    ax.clear()
    ax.plot(ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Distance over Time')
    plt.ylabel('Distance (cm)')
  

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=50)
plt.show()


