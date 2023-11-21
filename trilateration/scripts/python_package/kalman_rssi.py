import math

class FiltroKalmanRSSI:
    """
    Filtro de Kalman orientado a las mediciones RSSI en una posición estática.
    """
    """
    P: Matriz de covarianza del error de la estimación
    mu: Estimación del estado
    Inicio con valor nulo
    """
    P = float('nan')   
    mu = float('nan')

    def __init__(self, Q, R):
        """
        Constructor:
        Q: Ruido del proceso
        R: Ruido de la medición
        """
        self.A = 1
        self.B = 0
        self.C = 1

        self.Q = Q
        self.R = R
    
    def filtrar(self,rssi):
        """
        Filtrado de las mediciones del RSSI:
        rssi: Mediciones del RSSI a filtrar
        return: Devuelve los valores filtrados  
        """
        u = 0
        if math.isnan(self.mu):
            self.mu = (1 / self.C) * rssi
            self.P = (1 / self.C) * self.R * (1 / self.C)
        else:
            #Predicciones
            pred_mu = (self.A * self.mu) + (self.B * u)
            pred_P = self.A * self.P * self.A + self.Q
            #Ganancia de Kalman
            K = pred_P * self.C * (self.C * pred_P * self.C + self.R)**(-1)
            #Actualización
            self.mu = pred_mu + K*(rssi - self.C*pred_mu)
            self.P = pred_P - K * self.C * pred_P
        
        return self.mu
    
    def ajustar_ruido_proceso(self,ruido):
        """
        Ajustar la varianza del ruido del proceso Q
        """
        self.Q = ruido

    def ajustar_rudio_medicion(self, ruido):
        """
        Ajustar la varianza del ruido de medición R
        """
        self.R = ruido