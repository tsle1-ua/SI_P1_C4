"""
Módulo con implementaciones de:
 - A* clásico (w=1.0)
 - Weighted A* (0<=w<=1)
 - A*ε (epsilon-parametrizado)
"""
import heapq
import math
from typing import Tuple, List, Optional

class Nodo:
    """
    Nodo para los algoritmos A*.
    Attributes:
        posicion: coordenadas (fila, col)
        padre: nodo padre en la ruta
        g: coste acumulado desde el inicio
        h: heurística estimada al objetivo
        f: valor total según weight: (1-w)*g + w*h
        calorias: calorías consumidas hasta este nodo
    """
    def __init__(
        self,
        posicion: Tuple[int, int],
        padre: Optional['Nodo'] = None,
        w: float = 1.0
    ) -> None:
        self.posicion = posicion
        self.padre = padre
        self.g = 0.0
        self.h = 0.0
        self.f = 0.0
        self.calorias = 0
        self.w = w

    def __lt__(self, other: 'Nodo') -> bool:
        return self.f < other.f

def calcular_coste(
    pos1: Tuple[int,int],
    pos2: Tuple[int,int],
    mapa
) -> Tuple[float,int]:
    """Devuelve (mov_cost, terrain_calories) o (inf, inf) si bloqueado."""
    dr = abs(pos1[0]-pos2[0]); dc = abs(pos1[1]-pos2[1])
    movement_cost = 1.5 if dr==1 and dc==1 else 1.0
    terreno = mapa.getCelda(pos2[0], pos2[1])
    if terreno==0:
        terrain_calories = 2
    elif terreno==4:
        terrain_calories = 4
    elif terreno==5:
        terrain_calories = 6
    else:
        return float('inf'), float('inf')
    return movement_cost, terrain_calories

def heuristica(
    pos1: Tuple[int,int],
    pos2: Tuple[int,int]
) -> float:
    return math.hypot(pos1[0]-pos2[0], pos1[1]-pos2[1])

def obtener_vecinos(
    posicion: Tuple[int,int],
    mapa
) -> List[Tuple[int,int]]:
    moves = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
    vecinos=[]
    for dm, dn in moves:
        np = (posicion[0]+dm, posicion[1]+dn)
        if (0<=np[0]<mapa.getAlto() and 0<=np[1]<mapa.getAncho()
            and mapa.getCelda(np[0],np[1])!=1):
            vecinos.append(np)
    return vecinos

def reconstruir_camino(
    nodo: Nodo
) -> Tuple[List[Tuple[int,int]], int]:
    path=[]
    cal_tot = nodo.calorias
    while nodo:
        path.append(nodo.posicion)
        nodo = nodo.padre
    return path[::-1], cal_tot

def a_star(
    mapa,
    inicio: Tuple[int,int],
    objetivo: Tuple[int,int],
    w: float = 1.0
) -> Tuple[Optional[List[Tuple[int,int]]], Optional[int]]:
    """
    Weighted A*: f = (1-w)*g + w*h. Para w=1 es A* clásico.
    """
    start = Nodo(inicio, None, w)
    start.h = heuristica(inicio, objetivo)
    start.f = (1-w)*start.g + w*start.h
    frontier = []
    heapq.heappush(frontier, start)
    in_frontier = { inicio: start }
    closed = set()

    while frontier:
        current = heapq.heappop(frontier)
        if current.posicion in closed:
            continue
        closed.add(current.posicion)
        if current.posicion == objetivo:
            return reconstruir_camino(current)
        for vec in obtener_vecinos(current.posicion, mapa):
            if vec in closed:
                continue
            move_cost, cal = calcular_coste(current.posicion, vec, mapa)
            if move_cost==float('inf'):
                continue
            g_new = current.g + move_cost
            h_new = heuristica(vec, objetivo)
            f_new = (1-w)*g_new + w*h_new
            if vec in in_frontier:
                neigh = in_frontier[vec]
                if g_new < neigh.g:
                    neigh.g = g_new; neigh.h = h_new; neigh.f = f_new
                    neigh.padre = current; neigh.calorias = current.calorias+cal
                    heapq.heapify(frontier)
            else:
                neigh = Nodo(vec, current, w)
                neigh.g = g_new; neigh.h = h_new; neigh.f = f_new
                neigh.calorias = current.calorias+cal
                heapq.heappush(frontier, neigh)
                in_frontier[vec] = neigh
    return None, None

def a_star_epsilon(
    mapa,
    inicio: Tuple[int,int],
    objetivo: Tuple[int,int],
    epsilon: float = 0.1
) -> Tuple[Optional[List[Tuple[int,int]]], Optional[int]]:
    """
    A*ε: de la frontier construye una focal de nodos con f ≤ (1+ε)*f_min, elige el de mínimas calorías.
    """
    start = Nodo(inicio, None)
    start.h = heuristica(inicio, objetivo)
    start.f = start.g + start.h
    frontier = [start]
    in_frontier = { inicio: start }
    closed = set()

    while frontier:
        f_min = frontier[0].f
        focal = [n for n in frontier if n.f <= (1+epsilon)*f_min]
        current = min(focal, key=lambda x: x.calorias)
        frontier.remove(current)
        heapq.heapify(frontier)
        if current.posicion in closed:
            continue
        closed.add(current.posicion)
        if current.posicion == objetivo:
            return reconstruir_camino(current)
        for vec in obtener_vecinos(current.posicion, mapa):
            if vec in closed:
                continue
            move_cost, cal = calcular_coste(current.posicion, vec, mapa)
            if move_cost==float('inf'):
                continue
            g_new = current.g + move_cost
            h_new = heuristica(vec, objetivo)
            f_new = g_new + h_new
            if vec in in_frontier:
                neigh = in_frontier[vec]
                if g_new < neigh.g:
                    neigh.g = g_new; neigh.h = h_new; neigh.f = f_new
                    neigh.padre = current; neigh.calorias = current.calorias+cal
                    heapq.heapify(frontier)
            else:
                neigh = Nodo(vec, current)
                neigh.g = g_new; neigh.h = h_new; neigh.f = f_new
                neigh.calorias = current.calorias+cal
                heapq.heappush(frontier, neigh)
                in_frontier[vec] = neigh
    return None, None
