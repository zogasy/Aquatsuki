import cv2
import numpy as np

class Obstacle:
    def __init__(self):
        """
        Initialise l'objet Obstacle avec les limites HSV pour la détection.
        """
        self.bas_gris = np.array([0, 0, 50])     # H = 0, S = 0 (faible saturation), V minimum pour éviter le noir
        self.haut_gris = np.array([50, 50, 200])
        

    def detect(self, image):
        """
        Détecte les obstacles dans une image donnée.

        :param image: Image en format BGR (OpenCV)
        :return: Masque binaire et contours des obstacles détectés
        """
        # Convertir en HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Appliquer le filtre pour détecter les obstacles
        mask = cv2.inRange(hsv, self.bas_gris, self.haut_gris)

        # Détection des contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        return mask, contours

    def draw_obstacles(self, image, contours):
        """
        Dessine les contours des obstacles détectés sur l'image.
        
        :param image: Image originale (modifiable directement)
        :param contours: Liste des contours à dessiner
        :return: Image avec les contours dessinés
        """
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 100:  # Ignorer les petits objets
                # Dessiner le contour
                cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)
        return image
    
    def measure_obstacle_width(self, contour, image):
        """
        Mesure la largeur de l'obstacle (plus grand côté du rectangle) et dessine le rectangle autour.

        :param contour: Contour de l'obstacle à analyser
        :param image: Image dans laquelle dessiner le rectangle
        :return: Largeur de l'obstacle et image avec le rectangle dessiné
        """
        # Vérifier si le contour est valide
        if len(contour) >= 5:  
            # Calculer le rectangle minimal qui entoure le contour
            rect = cv2.minAreaRect(contour)
        
            (width, height) = rect[1]  

            # Dessin sur l'image
            box_points = cv2.boxPoints(rect)
            box_points = np.int0(box_points)  # Convertir les points en entiers
            cv2.drawContours(image, [box_points], 0, (0, 0, 255), 2)  # Dessiner le rectangle en rouge

            return max(width, height), image
        else:
            return 0, image  # Si le contour est trop petit, retourner 0 comme largeur et l'image sans modification
    

