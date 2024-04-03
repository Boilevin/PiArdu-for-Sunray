import matplotlib.pyplot as plt 
import numpy as np 
  
x = np.arange(10) 
y = [2, 4, 6, 14, 15, 16, 17, 
     16, 18, 20] 
y2 = [10, 11, 12, 13, 8, 10,  
      12, 14, 18, 19] 
  
fig, ax = plt.subplots() 
  
ax.plot(x, y, "go-", label ='Line 1', ) 
ax.plot(x, y2, "o-", label ='Line 2') 
  
chartBox = ax.get_position() 
ax.set_position([chartBox.x0+0.05, chartBox.y0, 
                 chartBox.width, 
                 chartBox.height * 0.6]) 
  
ax.legend(loc ='upper center', 
          bbox_to_anchor =(0.5, 1.45), 
          shadow = True, ncol = 1) 
   
fig.suptitle('matplotlib.axes.Axes.set_position()function Example', fontweight ="bold") 
plt.show() 