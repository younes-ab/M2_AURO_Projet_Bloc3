## Projet BLOC 3
# ABDESSALEM Younes, HAUSS Katell

import matplotlib.pyplot as plt
import numpy as np
import math


### Fonction

# Affichage
def affichage(pos_a: float, r: float, x_maj: float, t: int):
    # Affichage de la map
    for i in range(0, pos_a.shape[0], 2):
        plt.scatter(pos_a[i], pos_a[i + 1])

    # Affichage de la trajectoire du robot dans la map
    plt.plot(r[:t+1, 0], r[:t+1, 1])
    # Affichage du robot dans son dernier etat
    plt.scatter(r[t, 0], r[t, 1])  # remplacer t par le dernier indice du vecteur Temps
    # Affichage des états robot
    plt.plot(x_maj[:t+1, 0], x_maj[:t+1, 1])
    # Affichage du robot dans son dernier etat predit
    plt.scatter(x_maj[t, 0], x_maj[t, 1])

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

# Calcul position robot
def avance_robot(r : float, u : float, w : float, position_amer : float, t : float):
    if (u[t, 1] == 0):
        r[t+1, 0] = r[t, 0] + (u[t, 0]) * math.cos(r[t, 2]) + w[0,0]
        r[t+1, 1] = r[t, 1] + (u[t, 0]) * math.sin(r[t, 2]) + w[0,1]
        r[t+1, 2] = r[t, 2] + u[t, 1] + w[0,2]
        r[t+1, 3:] = position_amer
    else:
        r[t+1, 0] = r[t, 0] + (u[t, 0] / u[t, 1]) * (math.sin(r[t, 2] + u[t, 1]) - math.sin(r[t, 2])) + w[0,0]
        r[t+1, 1] = r[t, 1] + (u[t, 0] / u[t, 1]) * (math.cos(r[t, 2]) - math.cos(r[t, 2] + u[t, 1])) + w[0,1]
        r[t+1, 2] = r[t, 2] + u[t, 1] + w[0,2]
        r[t+1, 3:] = position_amer
    return r

# Generation trajectoire
def generation_trajectoire(nb_amer : float, x : float, position_amers : float, u : float, w : float, x_r : float,
                           y_r : float, theta_r : float, t : float):
    i = 0
    x[0, :3] = [x_r, y_r, theta_r]
    dist = x[0, 0] - position_amers[nb_amer]  # x_robot - x_amer_(4)

    # Aller en x
    while(dist < 0):
        u[t] = np.array([0.5, 0])
        x = avance_robot(x, u, w, position_amers, t)
        dist = x[i, 0] - position_amers[nb_amer]
        i += 1
        t += 1

    # Rotation
    while (x[i, 2] < np.pi * 0.95):
        u[t] = np.array([0.2, np.pi/10])
        x = avance_robot(x, u, w, position_amers, t)
        i += 1
        t += 1

    # Retour en x
    dist = x[i, 0] - position_amers[0]
    while (dist > 0):
        u[t] = np.array([0.5, 0])
        x = avance_robot(x, u, w, position_amers, t)
        dist = x[i, 0] - position_amers[0]
        i += 1
        t += 1

    # Rotation
    while (x[i, 2] < np.pi * 2 * 0.95):
        u[t] = np.array([0.2, np.pi/10])
        x = avance_robot(x, u, w, position_amers, t)
        i += 1
        t += 1

    # Aller 2 en x
    dist = x[i, 0] - position_amers[2]
    while (dist < 0 ):
        u[t] = np.array([0.5, 0])
        x = avance_robot(x, u, w, position_amers, t)
        dist = x[i, 0] - position_amers[2]
        i += 1
        t += 1
    return t, x, u



# Generation mesures
def generation_mesure(nb_amer : float, z1:float, r1:float, pos_a:float, v:float, t : float):
    for i in range(t):
        for e in range(nb_amer):
            z1[i, e*2] = math.sqrt((pos_a[2 * e] - r1[i, 0]) ** 2 + (pos_a[2 * e + 1] - r1[i, 1]) ** 2) + v[i,2*e]
            z1[i, 2*e+1] = (math.atan2(pos_a[2*e+1] - r1[i,1], pos_a[2*e] - r1[i,0]) - r1[i,2] + v[i,e+1]) %(np.pi)

            if (z1[i, e * 2] > 3 or abs(z1[i, e * 2 + 1]) > 3*np.pi/4 ):
                z1[i, e * 2] = np.nan
                z1[i, e * 2 + 1] = np.nan
        #print("z1[t] : ", z1[i])
    return z1

# Filtre
def filtre(x_pred : float, P_pred : float, x_maj : float, P_maj : float, K : float, z : float, u : float,
           position_amer : float, nb_amer : float, t : float):
    Qw_pred = np.diag(0.000000001 * np.ones(3 + 2 * nb_amer))
    for i in range(t):
        print("\nInstant n°", i)

        # Recup obs sans 'nan'
        amers_visibles = []
        for j in range(nb_amer):
            if (np.isnan(z[i,2 * j]) == False):
                amers_visibles = amers_visibles + [j]
        print("amers_visibles : ", amers_visibles)

        # Prediction
        if (u[i, 1] == 0):
            x_pred[i, 0] = x_maj[i, 0] + u[i, 0] * math.cos(x_maj[i, 2])
            x_pred[i, 1] = x_maj[i, 1] + u[i, 0] * math.sin(x_maj[i, 2])
            x_pred[i, 2] = x_maj[i, 2] + u[i, 1]
            x_pred[i, 3:] = position_amer
        else:
            x_pred[i, 0] = x_maj[i, 0] + (u[i, 0] / u[i, 1]) * (math.sin(x_maj[i, 2] + u[i, 1]) - math.sin(x_maj[i,2]))
            x_pred[i, 1] = x_maj[i, 1] + (u[i, 0] / u[i, 1]) * (math.cos(x_maj[i, 2]) - math.cos(x_maj[i, 2] + u[i, 1]))
            x_pred[i, 2] = x_maj[i, 2] + u[i, 1]
            x_pred[i, 3:] = position_amer

        F = jacF(x_pred, u, i)

        P_pred = F @ P_maj @ F.T + Qw_pred

        z_visible = np.zeros((1, len(amers_visibles * 2)))
        z_pred = np.zeros((1, len(amers_visibles * 2)))
        H = np.zeros((len(amers_visibles * 2), 19))
        Rv = np.zeros((len(amers_visibles * 2), len(amers_visibles * 2)))
        for o in range(len(amers_visibles)):
            Rv[2 * o][2 * o] = 0.01
            Rv[2 * o + 1][2 * o + 1] = np.pi / 10
        Rv = np.diag(0.0001 * np.ones(len(amers_visibles * 2)))                                        # a checker

        for e in range(len(amers_visibles)):
            z_visible[0, e * 2] = z[i,2 * amers_visibles[e]]
            z_visible[0, 2 * e + 1] = z[i,2 * amers_visibles[e] + 1]
            z_pred[0, e * 2] = math.sqrt((x_pred[i, 0] - position_amer[amers_visibles[e] - 1]) ** 2
                                         + (x_pred[i, 1] - position_amer[amers_visibles[e]]) ** 2)
            z_pred[0, 2 * e + 1] = math.atan2(position_amer[amers_visibles[e]] - x_pred[i, 1],
                                              position_amer[amers_visibles[e] - 1] - x_pred[i, 0]) - x_pred[i, 2]


            H[2*e, 0] = -2 * (position_amer[amers_visibles[e]] - x_pred[i, 0]) * 1 / (2 * math.sqrt(
                (x_pred[i, 0] - position_amer[2 * amers_visibles[e]]) ** 2 + (
                            x_pred[i, 1] - position_amer[2 * amers_visibles[e] + 1]) ** 2))
            H[2*e, 1] = -2 * (position_amer[amers_visibles[e] + 1] - x_pred[i, 1]) / (2 * math.sqrt(
                (x_pred[i, 0] - position_amer[2 * amers_visibles[e]]) ** 2 + (
                            x_pred[i, 1] - position_amer[2 * amers_visibles[e] + 1]) ** 2))
            H[2*e, 3+2*amers_visibles[e]] = 2 * (position_amer[amers_visibles[e]] - x_pred[i, 0]) / (2 * math.sqrt(
                (x_pred[i, 0] - position_amer[2 * amers_visibles[e]]) ** 2 + (
                            x_pred[i, 1] - position_amer[2 * amers_visibles[e] + 1]) ** 2))
            H[2*e, 4+2*amers_visibles[e]] = 2 * (position_amer[amers_visibles[e] + 1] - x_pred[i, 1]) / (2 * math.sqrt(
                (x_pred[i, 0] - position_amer[2 * amers_visibles[e]]) ** 2 + (
                            x_pred[i, 1] - position_amer[2 * amers_visibles[e] + 1]) ** 2))
            #
            H[2*e+1, 0] = (position_amer[amers_visibles[e] + 1] - x_pred[i, 2]) / (
                        (position_amer[amers_visibles[e]] - x_pred[i, 1]) ** 2 + (
                            position_amer[amers_visibles[e] + 1] - x_pred[i, 2]) ** 2)
            H[2*e+1, 1] = -(position_amer[amers_visibles[e]] - x_pred[i, 1]) / (
                        (position_amer[amers_visibles[e]] - x_pred[i, 1]) ** 2 + (
                            position_amer[amers_visibles[e] + 1] - x_pred[i, 2]) ** 2)
            H[2*e+1, 2] = -1
            H[2*e+1, 3+2*amers_visibles[e]] = -(position_amer[amers_visibles[e] + 1] - x_pred[i, 2]) / (
                        (position_amer[amers_visibles[e]] - x_pred[i, 1]) ** 2 + (
                            position_amer[amers_visibles[e] + 1] - x_pred[i, 2]) ** 2)
            H[2*e+1, 4+2*amers_visibles[e]] = (position_amer[amers_visibles[e]] - x_pred[i, 1]) / (
                        (position_amer[amers_visibles[e]] - x_pred[i, 1]) ** 2 + (
                            position_amer[amers_visibles[e] + 1] - x_pred[i, 2]) ** 2)

        S = Rv + H @ P_pred @ np.transpose(H)

        # Mise a jour
        K = P_pred @ H.T @ np.linalg.inv(S)

        x_maj = x_pred + K @ (z_visible[0] - z_pred[0])

        P_maj = P_pred - K @ H @ P_pred
    return  x_maj


# Calcul de F
def jacF(x_pred : float, u : float, i : int):
    F_sa = np.zeros((19, 19))
    if (u[i, 1] == 0):
        F_sa[0, 0] = 1
        F_sa[0, 2] = -u[i, 0] * math.sin(x_pred[i, 2])
        F_sa[1, 1] = 1
        F_sa[1, 2] = u[i, 0] * math.cos(x_pred[i, 2])
        F_sa[2, 2] = 1
    else:
        F_sa[0, 0] = 1
        F_sa[0, 2] = (u[i, 0] / u[i, 1]) * (math.cos(x_pred[i, 2] + u[i, 1]) - math.cos(x_pred[i, 2]))
        F_sa[1, 1] = 1
        F_sa[1, 2] = (u[i, 0] / u[i, 1]) * (math.sin(x_pred[i, 2] + u[i, 1]) - math.sin(x_pred[i, 2]))
        F_sa[2, 2] = 1
    return F_sa





### Main
def main():
    # Initialisation
    t = 0
    x_r = 0
    y_r = 0
    theta_r = 0
    nb_amer = 8
    x = np.zeros((100,19))
    u = np.zeros((100,2))
    z = np.zeros((100,16))
    Qw = np.diag([0.0000001, 0.0000001, 0.0000001])
    w = np.transpose((np.linalg.cholesky(Qw)) @ (np.random.normal(size=(3, 100))))               # a verifier math
    Rv = np.diag(0.000001 * np.ones(2 * nb_amer))
    v = np.transpose((np.linalg.cholesky(Rv)) @ (np.random.normal(size=(2 * nb_amer, 100))))     # a verifier math

    # Init Filtre
    x_pred = np.zeros((100, 19))
    P_pred = np.zeros((19, 19))
    x_maj = np.zeros((100, 19))
    P_maj = np.zeros((19, 19))
    K = np.zeros((19, 16))

    incertitude_amer = np.diag(0.0001 * np.ones(2 * nb_amer))                    # attention, a reverifier

    position_amer = generation_amers(nb_amer, 1, -1, incertitude_amer)
    t, x, u = generation_trajectoire(nb_amer, x, position_amer, u, w, x_r, y_r, theta_r, t)
    z = generation_mesure(nb_amer, z, x, position_amer, v, t)

    x_maj = filtre(x_pred, P_pred, x_maj, P_maj, K, z, u, position_amer, nb_amer, t)

    affichage(position_amer, x, x_maj, t)



if __name__ == "__main__":
    main()
