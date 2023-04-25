## Projet BLOC 3
# ABDESSALEM Younes, HAUSS Katell

import matplotlib.pyplot as plt
import numpy as np
import math


### Fonction

# Affichage
def affichage(pos_a: float, r: float, t: int):
    # Affichage de la map
    for i in range(0, pos_a.shape[0], 2):
        plt.scatter(pos_a[i], pos_a[i + 1])

    # Affichage de la trajectoire du robot dans la map
    plt.plot(r[:, 0], r[:, 1])
    # Affichage du robot dans son dernier etat
    plt.scatter(r[t, 0], r[t, 1])  # remplacer t par le dernier indice du vecteur Temps
    # Affichage des états robot
    #plt.plot(r_maj[:, 0], r_maj[:, 1])
    # Affichage du robot dans son dernier etat predit
    #plt.scatter(r_maj[T, 0], r_maj[T, 1])

    plt.show()


# Generation amers
def generation_amers(Nb_amer : int, x : float, y : float, incertitude_amer : float) :
    m = np.ndarray((Nb_amer*2))

    for i in range(int(Nb_amer / 2)):
        m[i*2] = x
        m[i*2+1] = y
        x = x + 2
    i = i + 1
    x = x - 2
    y = y + 2
    while i < Nb_amer:
        m[i * 2] = x
        m[i * 2 + 1] = y
        x = x - 2
        i = i + 1

    m = m + (np.linalg.cholesky(incertitude_amer))@(np.random.normal(size=(16)))

    return m


# Generation mesures




# F-san






### Main
def main():
    # Initialisation
    t = 0
    x_r = 0
    y_r = 0
    theta_r = 0
    nb_amer = 8
    r = np.array([[x_r, y_r, theta_r]])
    # r1 = np.array([[x_r, y_r, theta_r]])
    u = np.array([[1, 0]])

    incertitude_amer = np.diag(0.0001 * np.ones(2 * nb_amer))                    # attention, a reverifier

    position_amers = generation_amers(nb_amer, 1, -1, incertitude_amer)



    affichage(position_amers, r, t)



if __name__ == "__main__":
    main()
