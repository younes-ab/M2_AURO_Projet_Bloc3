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

#
def avance_robot(r : float, u : float, w : float, t : float):
    if (u[t, 1] == 0):
        r[t+1, 0] = r[t, 0] + (u[t, 0]) * math.cos(r[t, 2]) + w[0,0]
        r[t+1, 1] = r[t, 1] + (u[t, 0]) * math.sin(r[t, 2]) + w[0,1]
        r[t+1, 2] = r[t, 2] + u[t, 1] + w[0,2]
    else:
        r[t+1, 0] = r[t, 0] + (u[t, 0] / u[t, 1]) * (math.sin(r[t, 2] + u[t, 1]) - math.sin(r[t, 2])) + w[0,0]
        r[t+1, 1] = r[t, 1] + (u[t, 0] / u[t, 1]) * (math.cos(r[t, 2]) - math.cos(r[t, 2] + u[t, 1])) + w[0,1]
        r[t+1, 2] = r[t, 2] + u[t, 1] + w[0,2]
    return (r)

# Generation trajectoire
def generation_trajectoire(nb_amer : float, r : float, position_amers : float, u : float, w : float, x_r : float,
                           y_r : float, theta_r : float, t : float):
    i = 0
    r[0,:] = [x_r, y_r, theta_r]
    dist = r[0, 0] - position_amers[nb_amer]  # x_robot - x_amer_(4)

    # Aller en x
    while(dist < 0):
        u[t] = np.array([1, 0])
        r = avance_robot(r, u, w, t)
        dist = r[i, 0] - position_amers[nb_amer]
        i += 1
        t += 1

    # Rotation
    while (r[i, 2] < np.pi * 0.95):
        u[t] = np.array([0.5 , np.pi/8])
        r = avance_robot(r, u, w, t)
        i += 1
        t += 1

    # Retour en x
    dist = r[i, 0] - position_amers[0]
    while (dist > 0):
        u[t] = np.array([1, 0])
        r = avance_robot(r, u, w, t)
        dist = r[i, 0] - position_amers[0]
        i += 1
        t += 1

    # Rotation
    while (r[i, 2] < np.pi * 2 * 0.95):
        u[t] = np.array([0.5 , np.pi/8])
        r = avance_robot(r, u, w, t)
        i += 1
        t += 1

    # Aller 2 en x
    dist = r[i, 0] - position_amers[2]
    while (dist < 0 or i < 38):
        u[t] = np.array([1, 0])
        r = avance_robot(r, u, w, t)
        dist = r[i, 0] - position_amers[2]
        i += 1
        t += 1
    return t, r, u



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
    r = np.zeros((39,3))
    u = np.zeros((39,2))
    Qw = np.diag([0.00001, 0.00001, 0.00001])
    w = np.transpose((np.linalg.cholesky(Qw)) @ (np.random.normal(size=(3, 39))))

    incertitude_amer = np.diag(0.0001 * np.ones(2 * nb_amer))                    # attention, a reverifier

    position_amers = generation_amers(nb_amer, 1, -1, incertitude_amer)
    t, r, u = generation_trajectoire(nb_amer, r, position_amers, u, w, x_r, y_r, theta_r, t)


    affichage(position_amers, r, t)



if __name__ == "__main__":
    main()
