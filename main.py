'''Autonomous Agent Movement: Seek, Arrive and Flee

Created for COS30002 AI for Games, Lab 05
By Clinton Woodward cwoodward@swin.edu.au

'''
from graphics import egi, KEY
from pyglet import window, clock
from pyglet.gl import *

from vector2d import Vector2D
from world import World
from agent import Agent, AGENT_MODES  # Agent with wander, alignment,
                                      #cohesion & separation


def on_mouse_press(x, y, button, modifiers):
    if button == 1:  # left
        world.target = Vector2D(x, y)


def on_key_press(symbol, modifiers):
    if symbol == KEY.P:
        world.paused = not world.paused

    #reset path
    elif symbol == KEY.R:
        for agent in world.agents:
            agent.path.clear()
    #add new agent
    elif symbol == KEY.N:
        world.agents.append(Agent(world))
        world.agents[-1].mode = world.agents[0].mode
        world.agents[-1].AlignmentFactor = world.agents[0].AlignmentFactor
        world.agents[-1].CohesionFactor = world.agents[0].CohesionFactor
        world.agents[-1].SeparationFactor = world.agents[0].SeparationFactor
        world.agents[-1].max_speed = world.agents[0].max_speed

    #speed change
    elif symbol == KEY.Q:
        for agent in world.agents:
            agent.max_speed += 10
    elif symbol == KEY.W:
        for agent in world.agents:
            agent.max_speed -= 10

        
    #change mode
    elif symbol in AGENT_MODES:
        for agent in world.agents:
            agent.mode = AGENT_MODES[symbol]


    #change alignment factor
    elif symbol == KEY.A:
        for agent in world.agents:
            agent.AlignmentFactor = agent.AlignmentFactor + 0.1
    elif symbol == KEY.S:
        for agent in world.agents:
            agent.AlignmentFactor = agent.AlignmentFactor - 0.1

    #change cohesion factor
    elif symbol == KEY.Z:
        for agent in world.agents:
            agent.CohesionFactor = agent.CohesionFactor + 0.1
    elif symbol == KEY.X:
        for agent in world.agents:
            agent.CohesionFactor = agent.CohesionFactor - 0.1
            
    #change separation factor
    elif symbol == KEY.C:
        for agent in world.agents:
            agent.SeparationFactor = agent.SeparationFactor + 0.1
    elif symbol == KEY.V:
        for agent in world.agents:
            agent.SeparationFactor = agent.SeparationFactor - 0.1
    

    # Toggle debug force line info on the agent
    elif symbol == KEY.I:
        for agent in world.agents:
            agent.show_info = not agent.show_info



def on_resize(cx, cy):
    world.cx = cx
    world.cy = cy


if __name__ == '__main__':

    # create a pyglet window and set glOptions
    win = window.Window(width=500, height=500, vsync=True, resizable=True)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    # needed so that egi knows where to draw
    egi.InitWithPyglet(win)
    # prep the fps display
    fps_display = clock.ClockDisplay()
    # register key and mouse event handlers
    win.push_handlers(on_key_press)
    win.push_handlers(on_mouse_press)
    win.push_handlers(on_resize)

    # create a world for agents
    world = World(500, 500)
    # add one agent
    world.agents.append(Agent(world))
    # unpause the world ready for movement
    world.paused = False

    world.agents[0].color = 'BLUE'

    while not win.has_exit:
        win.dispatch_events()
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        # show nice FPS bottom right (default)
        delta = clock.tick()
        world.update(delta)
        world.render()
        fps_display.draw()
        # swap the double buffer
        win.flip()

