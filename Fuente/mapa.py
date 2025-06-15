import os

class Mapa:
    """
    Carga mapas desde un fichero .txt en la carpeta Fuente.
    """
    def __init__(self, archivo: str) -> None:
        base = os.path.dirname(__file__)
        ruta = os.path.join(base, archivo)
        if not os.path.isfile(ruta):
            raise FileNotFoundError(f"Mapa no encontrado: {ruta}")
        self.mapa = []
        with open(ruta, 'r') as f:
            for linea in f:
                fila = []
                for ch in linea.strip():
                    if ch == '.': fila.append(0)
                    elif ch == '#': fila.append(1)
                    elif ch == '~': fila.append(4)
                    elif ch == '*': fila.append(5)
                if fila:
                    self.mapa.append(fila)
        self.alto = len(self.mapa)
        self.ancho = len(self.mapa[0]) if self.alto > 0 else 0

    def getAlto(self) -> int:
        return self.alto

    def getAncho(self) -> int:
        return self.ancho

    def getCelda(self, y: int, x: int) -> int:
        return self.mapa[y][x]

    def setCelda(self, y: int, x: int, v: int) -> None:
        self.mapa[y][x] = v
