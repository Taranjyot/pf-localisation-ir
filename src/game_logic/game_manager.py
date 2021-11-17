# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from geometry_msgs/Point.msg. Do not edit."""
import codecs
import random
import sys
import genpy
import struct


class Cord():
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Robot():
    def __init__(self, id, hidingSpot):
        self.id = id
        self.hiding_spot: HidingSpot = hidingSpot


class HidingSpot():
    ## X and Y coordinate and bool to check if the location is occupied
    def __init__(self, _coordinate: Cord, _location_occupied):
        self.coordinate = _coordinate
        self.location_occupied = _location_occupied

class GameManger():

    def __init__(self, hidingSpots):
        super(GameManger, self).__init__()
        self.players_list: list = []
        self.active_hiders: list = []
        self.seeker: int = 0
        self.no_of_players: int = 10
        self.hiding_spots: list = hidingSpots
        self.occupied_hiding_spots: list = []
        self._setup_hiding_spots([])

    def _get_list_of_active_players(self):
        return self.active_hiders

    def _setup_list_of_players(self):
        for i in range(self.no_of_players):
            rob_hiding_spot_index = random.randrange(0, len(self.hiding_spots))
            rob_hiding_spot = self.hiding_spots.pop(rob_hiding_spot_index)
            self.occupied_hiding_spots.append(rob_hiding_spot)
            self.players_list.append(Robot(i,rob_hiding_spot))

        self.seeker = self.players_list.pop(random.randrange(0, len(self.players_list)))
        self.active_hiders = self.players_list

    def _setup_hiding_spots(self, list_of_spots):
        self._hiding_spots = list_of_spots

    def _player_caught(self, playerId: int):
        for robot in self.active_hiders:
            if playerId == robot.id:
                print("Player", playerId, "Caught At")
                print(robot.hiding_spot.coordinate.x, ",", robot.hiding_spot.coordinate.y)
                self.active_hiders.pop(self.active_hiders.index(robot))
                break
                ## rose topic player eliminated
        self._check_if_game_over()
    def _check_if_game_over(self):
        if len(self.active_hiders) == 1:
            print("Game Over")
        else:
            print(len(self.active_hiders), "Players Still Left")

    def find_position_to_hide(self):
        hiding_spot = self.hiding_spots.pop(random.randrange(0, len(self.hiding_spots)))
        self.occupied_hiding_spots.append(hiding_spot)

    def restart_game(self):
        self.no_of_players


## Debug Code for Game Manager
hidingspots = []
for i in range(15):
    hidingspots.append(HidingSpot(Cord(random.randrange(0,602), random.randrange(0,602)), False))
print(len(hidingspots))
a = GameManger(hidingspots)
a._setup_list_of_players()
print("Seeker", a.seeker.id)
a._player_caught(4)
a._player_caught(6)
a._player_caught(2)



## 602
