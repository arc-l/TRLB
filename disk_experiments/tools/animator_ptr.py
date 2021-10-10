import pygame
import pygame.gfxdraw
#import lor_solver
import time
import math

pygame.init()
pygame.font.init() # you have to call this at the start, 
                   # if you want to use this module.
myfont = pygame.font.SysFont('Comic Sans MS', 40)

colors = [(220, 0, 0), (0, 220, 0), (0, 0, 220), (250, 120, 0)]


top_offset = 0
left_offset = 0
item_size = 60
lift_height = 80
num_item = 16
color = (0, 255, 0)
corner_radius = 4

def draw_rounded_rect(surface, rect, color, corner_radius):
    ''' Draw a rectangle with rounded corners.
    Would prefer this: 
        pygame.draw.rect(surface, color, rect, border_radius=corner_radius)
    but this option is not yet supported in my version of pygame so do it ourselves.

    We use anti-aliased circles to make the corners smoother
    '''
    if rect.width < 2 * corner_radius or rect.height < 2 * corner_radius:
        raise ValueError(f"Both height (rect.height) and width (rect.width) must be > 2 * corner radius ({corner_radius})")

    # need to use anti aliasing circle drawing routines to smooth the corners
    pygame.gfxdraw.aacircle(surface, rect.left+corner_radius, rect.top+corner_radius, corner_radius, color)
    pygame.gfxdraw.aacircle(surface, rect.right-corner_radius-1, rect.top+corner_radius, corner_radius, color)
    pygame.gfxdraw.aacircle(surface, rect.left+corner_radius, rect.bottom-corner_radius-1, corner_radius, color)
    pygame.gfxdraw.aacircle(surface, rect.right-corner_radius-1, rect.bottom-corner_radius-1, corner_radius, color)

    pygame.gfxdraw.filled_circle(surface, rect.left+corner_radius, rect.top+corner_radius, corner_radius, color)
    pygame.gfxdraw.filled_circle(surface, rect.right-corner_radius-1, rect.top+corner_radius, corner_radius, color)
    pygame.gfxdraw.filled_circle(surface, rect.left+corner_radius, rect.bottom-corner_radius-1, corner_radius, color)
    pygame.gfxdraw.filled_circle(surface, rect.right-corner_radius-1, rect.bottom-corner_radius-1, corner_radius, color)

    rect_tmp = pygame.Rect(rect)

    rect_tmp.width -= 2 * corner_radius
    rect_tmp.center = rect.center
    pygame.draw.rect(surface, color, rect_tmp)

    rect_tmp.width = rect.width
    rect_tmp.height -= 2 * corner_radius
    rect_tmp.center = rect.center
    pygame.draw.rect(surface, color, rect_tmp)


def draw_bordered_rounded_rect(surface, rect, color, border_color, corner_radius, border_thickness):
    if corner_radius < 0:
        raise ValueError(f"border radius ({corner_radius}) must be >= 0")

    rect_tmp = pygame.Rect(rect)
    center = rect_tmp.center

    if border_thickness:
        if corner_radius <= 0:
            pygame.draw.rect(surface, border_color, rect_tmp)
        else:
            draw_rounded_rect(surface, rect_tmp, border_color, corner_radius)

        rect_tmp.inflate_ip(-2*border_thickness, -2*border_thickness)
        inner_radius = corner_radius - border_thickness + 1
    else:
        inner_radius = corner_radius

    if inner_radius <= 0:
        pygame.draw.rect(surface, color, rect_tmp)
    else:
        draw_rounded_rect(surface, rect_tmp, color, inner_radius)


def create_item_surface(num):
    spirit = pygame.Surface((60, 60),pygame.SRCALPHA, 32)
    c = 0
    if num in {3, 4, 7, 8}:
        c = 1
        
    if num in {9, 10,  13, 14}:
        c = 2
        
    if num in {11, 12, 15, 16}:
        c = 3
        
        
    draw_bordered_rounded_rect(spirit, pygame.Rect(5, 5, 50, 50), colors[c], (100, 100, 100), 10, 2)

    text_surf = myfont.render(str(num), True, (0, 0, 0))
    rect = text_surf.get_rect()
    spirit.blit(text_surf, (item_size/2-rect.width/2, item_size/2-rect.height/2))
    return spirit

def create_blank_surface():
    spirit =  pygame.Surface((60, 60),pygame.SRCALPHA, 32)
    spirit.set_alpha(128) 
    draw_bordered_rounded_rect(spirit, pygame.Rect(2, 2, 56, 56), (240, 240, 240), (150, 150, 150), 10, 2)
    return spirit

# Setting up a LOR instance
def get_items(plor):
    rects = []
    surfaces = []
    for i in range(len(plor)):
        col = i//4
        row = i%4
        rects.append(pygame.Rect(left_offset + col*item_size, top_offset + row*item_size , item_size,item_size))
        surfaces.append(create_item_surface(plor[i]+1))

    rects.append(pygame.Rect(left_offset, top_offset, item_size,item_size))
    surfaces.append(create_blank_surface())

    return (rects, surfaces)

plor = [6, 13, 5, 3, 9, 7, 12, 4, 14, 2, 1, 11, 10, 8, 15, 0]
num_item = len(plor)


(rects, surfaces) = get_items(plor)
#plan = [[0, -1, 6], [ 2, 6, 5], [1, 5, 13], [12, 13, 10], [15, 10, 0], [4, 0, 9], [8, 9, 14], [10, 14, 1], [5, 1, 7], [6, 7, 12], 
#        [9, 12, 2], [7, 2, 4], [0, 4, -1]]
# [[nextlocation, startItem, endItem]]
plan = [[0, 1, 6], [2, 6, 5], [0, 5, -1], [1, -1, 13],[8, 13, 14], [10, 14, 1], [4, 1, 9], [9, 9, 2], [7, 2, 4], [1, 4, -1],
        [5, -1, 7], [6, 7, 12], [12, 12, 10], [15, 10, 0], [5, 0, -1], [0, -1, -1]]

# Set up the drawing window
screen = pygame.display.set_mode([240, 240])
clock = pygame.time.Clock()


step = []
location = -1
startItem = -1
endItem = -1 
stepRunning = False
stepOne = False

lastLocation = 0
countstart = 0
count = 0
sx = 0
sy = 0
dx = 0
dy = 0
lx = 0
ly = 0

step_size = 5
step_size2 = 1

# Compute a inverse array that gives locations of items
iis = [0 for i in range(len(plor))]
for i in range(len(plor)):
    iis[plor[i]] = i
iis.append(len(plor))

firstRun = True
sleepABit = False

# Run until the user asks to quit
ts2 = time.time()
running = True
while running:

    # Did the user click the window close button?
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Fill the background with white
    screen.fill((255, 255, 255))

    # Draw a solid blue circle in the center
    for i in range(len(rects) - 1):
        if i != iis[startItem]:
            screen.blit(surfaces[i], (rects[i].left, rects[i].top))
    screen.blit(surfaces[iis[startItem]], (rects[iis[startItem]].left, rects[iis[startItem]].top))        
    screen.blit(surfaces[len(rects) - 1], (rects[len(rects) - 1].left, rects[len(rects) - 1].top))

    # Flip the display
    pygame.display.flip()

    if sleepABit:
        time.sleep(0.2)
        sleepABit = False


    # Update rectangle locations
    if not stepRunning:
        if len(plan) > 0:
            step = plan.pop(0)
            nextLocation = step[0]
            startItem = num_item if step[1] == -1 else step[1]
            endItem = num_item if step[2] == -1 else step[2]

            # Prepare step one (swap)
            lx = lastLocation // 4
            ly = lastLocation % 4
            nx = nextLocation // 4
            ny = nextLocation % 4
            sx = rects[iis[startItem]].left
            sy = rects[iis[startItem]].top

            dist = math.sqrt((nx - lx)*(nx - lx) + (ny - ly)*(ny-ly))

            if dist == 0: 
                continue

            countstart = count = round(dist*item_size)//step_size

            dx = (nx - lx)*step_size/(dist)
            dy = (ny - ly)*step_size/(dist)

            lastLocation = nextLocation

            lx = lastLocation // 4
            ly = lastLocation % 4

            stepOne = True
            stepRunning = True

        else:
            running = False
            time.sleep(3)
            continue            

    if stepOne:
        if count > 0:
            count = count -  1
            rects[iis[startItem]].left = round(left_offset + lx*item_size - count*dx)
            rects[iis[startItem]].top = round(top_offset + ly*item_size - count*dy)
            rects[len(rects)-1].left = round(left_offset + lx*item_size - count*dx)
            rects[len(rects)-1].top = round(top_offset + ly*item_size - count*dy)
        else:
            rects[iis[startItem]].left = left_offset + lx*item_size
            rects[iis[startItem]].top = top_offset + ly*item_size
            rects[len(rects)-1].left = left_offset + lx*item_size
            rects[len(rects)-1].top = top_offset + ly*item_size
            stepOne = False
            stepRunning = False
            countstart = 0
            sleepABit = True
            

    # Ensure program maintains a rate of 30 frames per second
    clock.tick(30)

    if firstRun:
        time.sleep(3)
        ts2 = time.time()
        firstRun = False

# Done! Time to quit.

print(time.time() - ts2)

pygame.quit()