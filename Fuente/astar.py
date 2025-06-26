"""
Módulo con implementaciones de:
 - A* clásico (w=1.0)
 - Weighted A* (0<=w<=1)
 - A*ε (epsilon-parametrizado)
"""
import heapq
import math
from typing import Tuple, List, Optional, Set, Dict

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
    
    # Mapa de terreno a calorías
    calorias_terreno = {
        0: 2,  # Hierba
        4: 4,  # Agua
        5: 6   # Roca
    }
    
    if terreno in calorias_terreno:
        terrain_calories = calorias_terreno[terreno]
    else:
        return float('inf'), float('inf')
    
    return movement_cost, terrain_calories

def heuristica(
    pos1: Tuple[int,int],
    pos2: Tuple[int,int]
) -> float:
    """Distancia euclídea - heurística admisible para movimiento en 8 direcciones."""
    return math.hypot(pos1[0]-pos2[0], pos1[1]-pos2[1])

def heuristica_manhattan(
    pos1: Tuple[int,int],
    pos2: Tuple[int,int]
) -> float:
    """Distancia Manhattan - no admisible para movimiento diagonal."""
    return abs(pos1[0]-pos2[0]) + abs(pos1[1]-pos2[1])

def heuristica_chebyshev(
    pos1: Tuple[int,int],
    pos2: Tuple[int,int]
) -> float:
    """Distancia Chebyshev - admisible para movimiento en 8 direcciones con coste uniforme."""
    return max(abs(pos1[0]-pos2[0]), abs(pos1[1]-pos2[1]))

def obtener_vecinos(
    posicion: Tuple[int,int],
    mapa
) -> List[Tuple[int,int]]:
    """Obtiene vecinos válidos en las 8 direcciones."""
    moves = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
    vecinos = []
    for dm, dn in moves:
        np = (posicion[0]+dm, posicion[1]+dn)
        if (0 <= np[0] < mapa.getAlto() and
            0 <= np[1] < mapa.getAncho() and
            mapa.getCelda(np[0], np[1]) != 1):
            vecinos.append(np)
    return vecinos

def reconstruir_camino(
    nodo: Nodo
) -> Tuple[List[Tuple[int,int]], int]:
    """Reconstruye el camino desde el nodo objetivo hasta el inicio."""
    path = []
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
    Weighted A*: f = (1-w)*g + w*h. 
    Para w=1 es A* clásico (solo heurística).
    Para w=0 es búsqueda de coste uniforme.
    """
    # Validar parámetros
    if not (0 <= w <= 1):
        raise ValueError("El peso w debe estar entre 0 y 1")
    
    # Verificar que inicio y objetivo son transitables
    if (mapa.getCelda(inicio[0], inicio[1]) == 1 or 
        mapa.getCelda(objetivo[0], objetivo[1]) == 1):
        return None, None
    
    start = Nodo(inicio, None, w)
    start.h = heuristica(inicio, objetivo)
    start.f = (1-w)*start.g + w*start.h
    
    frontier = []
    heapq.heappush(frontier, start)
    in_frontier = { inicio: start }
    closed = set()
    
    # Contador de nodos explorados (para debugging)
    nodos_explorados = 0

    while frontier:
        current = heapq.heappop(frontier)
        
        # Si ya fue explorado, continuar
        if current.posicion in closed:
            continue
            
        closed.add(current.posicion)
        nodos_explorados += 1

        # Verificar si llegamos al objetivo
        if current.posicion == objetivo:
            print(f"A* (w={w}): Nodos explorados: {nodos_explorados}")
            return reconstruir_camino(current)

        # Explorar vecinos
        for vec in obtener_vecinos(current.posicion, mapa):
            move_cost, cal = calcular_coste(current.posicion, vec, mapa)
            if move_cost == float('inf'):
                continue
                
            g_new = current.g + move_cost
            h_new = heuristica(vec, objetivo)
            f_new = (1-w)*g_new + w*h_new

            if vec in in_frontier:
                neigh = in_frontier[vec]
                if g_new < neigh.g:
                    neigh.g = g_new
                    neigh.h = h_new
                    neigh.f = f_new
                    neigh.padre = current
                    neigh.calorias = current.calorias + cal
                    heapq.heapify(frontier)
            else:
                neigh = Nodo(vec, current, w)
                neigh.g = g_new
                neigh.h = h_new
                neigh.f = f_new
                neigh.calorias = current.calorias + cal
                heapq.heappush(frontier, neigh)
                in_frontier[vec] = neigh

    print(f"A* (w={w}): No se encontró solución. Nodos explorados: {nodos_explorados}")
    return None, None

def a_star_epsilon(
    mapa,
    inicio: Tuple[int,int],
    objetivo: Tuple[int,int],
    epsilon: float = 0.1
) -> Tuple[Optional[List[Tuple[int,int]]], Optional[int]]:
    """
    A*ε: focal de nodos con f ≤ (1+ε)*f_min, elige el de menor calorías.
    """
    # Validar parámetros
    if epsilon < 0:
        raise ValueError("Epsilon debe ser >= 0")
    
    # Verificar que inicio y objetivo son transitables
    if (mapa.getCelda(inicio[0], inicio[1]) == 1 or 
        mapa.getCelda(objetivo[0], objetivo[1]) == 1):
        return None, None
    
    start = Nodo(inicio, None)
    start.h = heuristica(inicio, objetivo)
    start.f = start.g + start.h
    
    frontier = []
    heapq.heappush(frontier, start)
    in_frontier = { inicio: start }
    closed = set()
    
    # Contador de nodos explorados
    nodos_explorados = 0

    while frontier:
        # Encontrar f_min
        f_min = min(n.f for n in frontier)
        
        # Construir lista focal
        focal = [n for n in frontier if n.f <= (1+epsilon)*f_min]
        
        # Elegir el nodo con menos calorías de la lista focal
        current = min(focal, key=lambda x: x.calorias)
        
        # Remover de frontier
        frontier.remove(current)
        heapq.heapify(frontier)
        
        # Remover de in_frontier
        if current.posicion in in_frontier:
            del in_frontier[current.posicion]

        if current.posicion in closed:
            continue
            
        closed.add(current.posicion)
        nodos_explorados += 1

        if current.posicion == objetivo:
            print(f"A*ε (ε={epsilon}): Nodos explorados: {nodos_explorados}")
            return reconstruir_camino(current)

        for vec in obtener_vecinos(current.posicion, mapa):
            if vec in closed:
                continue
                
            move_cost, cal = calcular_coste(current.posicion, vec, mapa)
            if move_cost == float('inf'):
                continue
                
            g_new = current.g + move_cost
            h_new = heuristica(vec, objetivo)
            f_new = g_new + h_new

            if vec in in_frontier:
                neigh = in_frontier[vec]
                if g_new < neigh.g:
                    neigh.g = g_new
                    neigh.h = h_new
                    neigh.f = f_new
                    neigh.padre = current
                    neigh.calorias = current.calorias + cal
                    heapq.heapify(frontier)
            else:
                neigh = Nodo(vec, current)
                neigh.g = g_new
                neigh.h = h_new
                neigh.f = f_new
                neigh.calorias = current.calorias + cal
                heapq.heappush(frontier, neigh)
                in_frontier[vec] = neigh

    print(f"A*ε (ε={epsilon}): No se encontró solución. Nodos explorados: {nodos_explorados}")
    return None, None

# Funciones de prueba para comparar heurísticas
def comparar_heuristicas(mapa, inicio, objetivo):
    """Compara diferentes heurísticas y muestra estadísticas."""
    print("\n=== Comparación de heurísticas ===")
    
    # h=0 (Búsqueda de coste uniforme)
    ruta, cal = a_star(mapa, inicio, objetivo, w=0.0)
    if ruta:
        print(f"h=0: Coste={len(ruta)-1}, Calorías={cal}")
    
    # Distancia Euclídea (por defecto)
    ruta, cal = a_star(mapa, inicio, objetivo, w=1.0)
    if ruta:
        print(f"Euclídea: Coste={len(ruta)-1}, Calorías={cal}")
    
    # A*ε con diferentes valores
    for eps in [0.1, 0.3, 0.5]:
        ruta, cal = a_star_epsilon(mapa, inicio, objetivo, epsilon=eps)
        if ruta:
            print(f"A*ε (ε={eps}): Coste={len(ruta)-1}, Calorías={cal}")