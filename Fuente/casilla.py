from dataclasses import dataclass

@dataclass
class Casilla:
    fila: int
    col: int

    def getFila(self) -> int:
        return self.fila

    def getCol(self) -> int:
        return self.col
