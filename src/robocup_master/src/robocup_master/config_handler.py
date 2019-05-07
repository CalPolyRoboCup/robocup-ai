'''
Handles reading and writing metadata to and from xml
'''
from robocup_master.config_data import Path, Pose, Config, Robot, Ball
import xml.etree.ElementTree as ET
from PyQt4.QtCore import Qt

# open a previously created trajectory file
def read_trajectory(f):
    tree = ET.parse(f)
    root = tree.getroot()
    path = Path()
    for child in root:
        if child.tag == 'point':
            pose = Pose()
            for loc in child:
                if loc.tag == 'x':
                    pose.x = float(loc.text)
                if loc.tag == 'y':
                    pose.y = float(loc.text)
                if loc.tag == 'theta':
                    pose.theta = float(loc.text)
            path.poses.append(pose)
    return path

# write a trajectory file
def write_trajectory(f, path):
    traj = ET.Element('traj')
    for p in path.poses:
        point = ET.SubElement(traj, 'point')
        p_x = ET.SubElement(point, 'x')
        p_x.text = str(p.x)
        p_y = ET.SubElement(point, 'y')
        p_y.text = str(p.y)
        p_theta = ET.SubElement(point, 'theta')
        p_theta.text = str(0.0)

    tree = ET.ElementTree(traj)
    tree.write(f)

# read a placement file
def read_placement(f):
    tree = ET.parse(f)
    root = tree.getroot()
    config = Config()
    for child in root:
        if child.tag == 'ball':
            ball = Ball()
            for ball_loc in child:
                if ball_loc.tag == 'x':
                    ball.x = float(ball_loc.text)
                if ball_loc.tag == 'y':
                    ball.y = float(ball_loc.text)
            config.ball = ball

        if child.tag == 'player':
            tmp_player = Robot()
            for player_pose in child:
                if player_pose.tag == 'x':
                    tmp_player.pose.x = float(player_pose.text)
                if player_pose.tag == 'y':
                    tmp_player.pose.y = float(player_pose.text)
                if player_pose.tag == 'theta':
                    tmp_player.pose.theta = float(player_pose.text)
                if player_pose.tag == 'color':
                    if player_pose.text == 'yellow':
                        tmp_player.color = Qt.yellow
                        config.robots_yellow.append(tmp_player)
                    elif player_pose.text == 'blue':
                        tmp_player.color = Qt.blue
                        config.robots_blue.append(tmp_player)
                    else:
                        print 'error invalid color'
    return config

def write_placement(f, config_md):
    config = ET.Element('config')
    for p in config_md.robots_yellow:
        player_loc = ET.SubElement(config, 'player')
        player_x = ET.SubElement(player_loc, 'x')
        player_x.text = str(p.pose.x)
        player_y = ET.SubElement(player_loc, 'y')
        player_y.text = str(p.pose.y)
        player_theta = ET.SubElement(player_loc, 'theta')
        player_theta.text = str(p.pose.theta)
        player_color = ET.SubElement(player_loc, 'color')
        player_color.text = 'yellow'

    for p in config_md.robots_blue:
        player_loc = ET.SubElement(config, 'player')
        player_x = ET.SubElement(player_loc, 'x')
        player_x.text = str(p.pose.x)
        player_y = ET.SubElement(player_loc, 'y')
        player_y.text = str(p.pose.y)
        player_theta = ET.SubElement(player_loc, 'theta')
        player_theta.text = str(p.pose.theta)
        player_color = ET.SubElement(player_loc, 'color')
        player_color.text = 'blue'
    
    if config_md.ball is not None:
        ball_loc = ET.SubElement(config, 'ball')
        b_x = ET.SubElement(ball_loc, 'x')
        b_x.text = str(config_md.ball.x)
        b_y = ET.SubElement(ball_loc, 'y')
        b_y.text = str(config_md.ball.y)

    tree = ET.ElementTree(config)
    tree.write(f)
