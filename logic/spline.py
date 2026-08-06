import numpy as np

class SimpleCubicSpline:
    """Implementación de Spline Cúbico Natural"""
    def __init__(self, x, y):
        self.x = np.asfarray(x)
        self.y = np.asfarray(y)
        n = len(x)
        h = np.diff(x)
        df = np.diff(y) / h
        
        # Configuración del sistema de ecuaciones tridiagonal
        A = np.zeros((n, n))
        B = np.zeros(n)
        
        A[0, 0] = 1
        A[n-1, n-1] = 1
        
        for i in range(1, n-1):
            A[i, i-1] = h[i-1]
            A[i, i] = 2 * (h[i-1] + h[i])
            A[i, i+1] = h[i]
            B[i] = 3 * (df[i] - df[i-1])
            
        # Resolver para los coeficientes c
        self.c = np.linalg.solve(A, B)
        self.b = df - h * (2 * self.c[:-1] + self.c[1:]) / 3
        self.d = (self.c[1:] - self.c[:-1]) / (3 * h)
        self.a = self.y[:-1]

    def __call__(self, x_new, der=0):
        if isinstance(x_new, (int, float)):
            x_new = np.array([x_new])
            
        # Encontrar a qué segmento pertenece x_new
        idx = np.searchsorted(self.x, x_new) - 1
        idx = np.clip(idx, 0, len(self.x) - 2)
        
        dx = x_new - self.x[idx]
        
        if der == 0:
            return self.a[idx] + self.b[idx]*dx + self.c[idx]*dx**2 + self.d[idx]*dx**3
        elif der == 1:
            return self.b[idx] + 2*self.c[idx]*dx + 3*self.d[idx]*dx**2
        return 0
