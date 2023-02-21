import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

def gen_potential_field(Goal_Loc):
    """
    This function takes in the current drone y position
    and generates a potential field for that y value
    Args:
        None
    Returns:
    Pot_field: a 2D array representing a potenial field
        for the drone to navigate
    Need a list of 4 item tuples, the 4 items are x,y,z, and yaw
    """
    x = np.linspace(-1, 2.5, 101)
    y = np.linspace(-1, 1, 101)
    xx, yy = np.meshgrid(x, y)
    zz = 0

    
    xlines_x = np.linspace(0,1.22,10)
    xlines_y = [0.605,0.3,-0.3,-0.605]
    for line1 in xlines_y:
      for point1 in xlines_x:
        zz=zz-np.log(np.sqrt((xx-point1)**2 + (yy-line1)**2))

    ylines1_y = np.linspace(0.3,1.22,3)
    ylines1_x = [0,1.22]
    for line2 in ylines1_y:
      for point2 in ylines1_x:
        zz=zz-np.log(np.sqrt((xx-line2)**2 + (yy-point2)**2))

    ylines2_y = np.linspace(-0.3,-1.22,3)
    ylines2_x = [0,1.22]
    for line3 in ylines2_y:
      for point3 in ylines2_x:
        zz=zz-np.log(np.sqrt((xx-line3)**2 + (yy-point3)**2))
    
    zz=zz+30*np.log(np.sqrt(abs(xx-Goal_Loc[0]) + abs(yy-Goal_Loc[1])))
    

    xx.shape, yy.shape, zz.shape
    # sparse coordinate arrays
    xs, ys = np.meshgrid(x, y, sparse=True)
    zs = np.sqrt(xs**2 + ys**2)
    xs.shape, ys.shape, zs.shape
    np.array_equal(zz, zs)

    fig = plt.figure(figsize =(14, 9))
    ax = plt.axes(projection ='3d')
    surf = ax.plot_surface(xx, yy, zz, cmap=cm.coolwarm,linewidth=0, antialiased=False)
    plt.show()

    h = plt.contourf(xx, yy, zz)
    plt.axis('scaled')
    plt.colorbar()
    plt.show()
    return [xx,yy,zz]

gen_potential_field([1.,0.6])