import sys
import os
import argparse
import pygame
from pygame.locals import *
from casilla import Casilla
from mapa import Mapa
from astar import a_star, a_star_epsilon

print("Iniciando Práctica A*...")

# Colores y constantes
MARGEN     = 5
MARGEN_INF = 80
TAM        = 30
NEGRO      = (0,0,0)
HIERBA     = (250,180,160)
MURO       = (30,70,140)
AGUA       = (173,216,230)
ROCA       = (110,75,48)
AMARILLO   = (255,255,0)

class Slider:
    def __init__(self, x, y, w, h, initial=0.8):
        self.rect       = pygame.Rect(x, y, w, h)
        self.handle_rad = h
        self.value      = initial
        self.dragging   = False

    def draw(self, screen):
        pygame.draw.rect(screen, AMARILLO, self.rect, 2)
        cx = self.rect.x + int(self.value * self.rect.w)
        cy = self.rect.y + self.rect.h // 2
        pygame.draw.circle(screen, AMARILLO, (cx, cy), self.handle_rad)

    def handle_event(self, ev):
        if   ev.type == MOUSEBUTTONDOWN and self.rect.collidepoint(ev.pos):
            self.dragging = True
        elif ev.type == MOUSEBUTTONUP:
            self.dragging = False
        elif ev.type == MOUSEMOTION and self.dragging:
            relx = ev.pos[0] - self.rect.x
            self.value = max(0.0, min(1.0, relx / self.rect.w))
        return False

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--w',    type=float, default=0.8,
                   help='Peso w para weighted A* (por defecto 0.8)')
    p.add_argument('--eps',  type=float, default=0.1,
                   help='Epsilon para A*ε (por defecto 0.1)')
    p.add_argument('mapfile', nargs='?', default='mapa.txt',
                   help='Fichero de mapa (.txt) en la carpeta Fuente')
    return p.parse_args()

def inic(mapa):
    """Inicializa la matriz camino con '.' en todas las celdas"""
    return [['.' for _ in range(mapa.getAncho())] for _ in range(mapa.getAlto())]

def main():
    args = parse_args()
    pygame.init()

    # Carga mapa usando ruta relativa a este script
    try:
        mapi = Mapa(args.mapfile)
    except Exception as e:
        print(e)
        sys.exit(1)

    # Dimensiones ventana
    ancho = mapi.getAncho() * (TAM+MARGEN) + MARGEN
    alto  = mapi.getAlto()  * (TAM+MARGEN) + MARGEN + MARGEN_INF
    screen = pygame.display.set_mode((ancho, alto))
    pygame.display.set_caption("Práctica Recuperación SI - A*")
    reloj = pygame.time.Clock()

    # Ruta base para cargar imágenes
    base = os.path.dirname(__file__)

    img_rabbit = pygame.transform.scale(
        pygame.image.load(os.path.join(base, "rabbit.png")), (TAM, TAM)
    )
    img_carrot = pygame.transform.scale(
        pygame.image.load(os.path.join(base, "carrot.png")), (TAM, TAM)
    )
    btn_astar = pygame.transform.scale(
        pygame.image.load(os.path.join(base, "boton1.png")), (60, 30)
    )
    btn_astep = pygame.transform.scale(
        pygame.image.load(os.path.join(base, "boton2.png")), (60, 30)
    )

    slider    = Slider(10, alto - MARGEN_INF + 20, ancho - 20, 8, initial=args.w)
    origen    = Casilla(-1, -1)
    destino   = Casilla(-1, -1)
    ruta, cal = [], 0
    coste     = -1
    algoritmo = None
    font      = pygame.font.Font(None, 24)
    
    # Inicializar matriz camino
    camino = inic(mapi)

    while True:
        for ev in pygame.event.get():
            if ev.type == QUIT:
                pygame.quit()
                return

            slider.handle_event(ev)

            if ev.type == MOUSEBUTTONDOWN:
                x, y = ev.pos
                # Botones
                if ancho//2-100 < x < ancho//2-40 and alto-50 < y < alto-20:
                    algoritmo = 'astar'
                if ancho//2+40  < x < ancho//2+100 and alto-50 < y < alto-20:
                    algoritmo = 'astep'
                # Selección en mapa
                if (MARGEN < x < ancho - MARGEN and
                    MARGEN < y < mapi.getAlto()*(TAM+MARGEN)+MARGEN):
                    c   = (x - MARGEN)//(TAM+MARGEN)
                    f   = (y - MARGEN)//(TAM+MARGEN)
                    cas = Casilla(f, c)
                    if ev.button == 1:
                        origen = cas
                    elif ev.button == 3:
                        destino = cas
                # Ejecutar algoritmo
                if algoritmo and origen.fila>=0 and destino.fila>=0:
                    # Reiniciar camino
                    camino = inic(mapi)
                    
                    if algoritmo == 'astar':
                        ruta, cal = a_star(
                            mapi,
                            (origen.fila, origen.col),
                            (destino.fila, destino.col),
                            slider.value
                        )
                    else:
                        ruta, cal = a_star_epsilon(
                            mapi,
                            (origen.fila, origen.col),
                            (destino.fila, destino.col),
                            args.eps
                        )
                    
                    # Actualizar matriz camino
                    if ruta:
                        for (f, c) in ruta:
                            camino[f][c] = 'X'
                        coste = len(ruta) - 1
                    else:
                        coste = -1

        # Dibujar
        screen.fill(NEGRO)
        for f in range(mapi.getAlto()):
            for c in range(mapi.getAncho()):
                v     = mapi.getCelda(f, c)
                color = MURO   if v==1 else \
                        HIERBA if v==0 else \
                        AGUA   if v==4 else \
                        ROCA
                pygame.draw.rect(screen, color,
                                 [(TAM+MARGEN)*c+MARGEN,
                                  (TAM+MARGEN)*f+MARGEN,
                                  TAM, TAM])
        
        # Dibujar camino usando la matriz camino
        for f in range(mapi.getAlto()):
            for c in range(mapi.getAncho()):
                if camino[f][c] != '.':
                    pygame.draw.rect(screen, AMARILLO,
                                     [(TAM+MARGEN)*c+MARGEN,
                                      (TAM+MARGEN)*f+MARGEN,
                                      TAM, TAM])
        
        if origen.fila>=0:
            screen.blit(img_rabbit,
                        [(TAM+MARGEN)*origen.col+MARGEN,
                         (TAM+MARGEN)*origen.fila+MARGEN])
        if destino.fila>=0:
            screen.blit(img_carrot,
                        [(TAM+MARGEN)*destino.col+MARGEN,
                         (TAM+MARGEN)*destino.fila+MARGEN])
        screen.blit(btn_astar, (ancho//2-100, alto-50))
        screen.blit(btn_astep, (ancho//2+40, alto-50))
        slider.draw(screen)
        screen.blit(font.render(f"w={slider.value:.2f}", True, AMARILLO),
                    (10, alto - MARGEN_INF))
        if coste >= 0:
            texto_coste = font.render(f"Coste: {coste:.1f}", True, AMARILLO)
            screen.blit(texto_coste, (ancho-150, alto - MARGEN_INF))
            texto_cal = font.render(f"Cal: {cal}", True, AMARILLO)
            screen.blit(texto_cal, (ancho-150, alto - MARGEN_INF + 25))
        pygame.display.flip()
        reloj.tick(30)

if __name__ == "__main__":
    main()