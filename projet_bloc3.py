## Projet BLOC 3
import matplotlib.pyplot as plt
import numpy as np


## Definir les amers
def amers(Nb_amer : int, x0 : float, y0 : float) :
    x = x0
    y = y0
    m = np.ndarray((Nb_amer, 2))
    
    xa = np.ndarray(Nb_amer)
    ya = np.ndarray(Nb_amer)
    if (Nb_amer % 2 == 0) :
        for i in range(int(Nb_amer / 2 )) :
            m[i] = [x, y]
            x = x + 2
        i = i + 1
        x = x - 2
        y = y - 2
        while i < Nb_amer :
            m[i] = [x, y]
            x = x - 2
            i = i + 1
    else :
        for i in range(int(Nb_amer / 2)) :
            m[i] = [x, y]
            x = x + 2
        i = i + 1
        y = y - 1
        m[i] = [x, y]
        
        i = i + 1
        x = x - 2
        y = y - 1
        while (i < Nb_amer) :
            m[i] = [x, y]
            x = x - 2
            i = i + 1
        
    xa = m[:,0]
    ya = m[:,1]
    return (xa,ya)

#def trajectoire(xr : float, yr : float, thetar : float) :




## Affichage de la map
(xa,ya)=amers(8,0,1)
plt.scatter(xa,ya)
plt.show()