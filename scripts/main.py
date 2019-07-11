#!/usr/bin/python
import os, sys
#This is necessary to import code that is not rosbased in the main.py
path = os.path.abspath(__file__)
dir_path = os.path.dirname(path)
parent_dir_of_file = os.path.dirname(dir_path)
parent_parent_dir_of_file = os.path.dirname(parent_dir_of_file)

from datetime import datetime
import csv

#ros import
import time
import rospy
from std_msgs.msg import String, Int8

from iri_rfid_board_scanner.msg import BoardIds
from board_state.msg import StrList
class GameFramework():

  def __init__(self):
    rospy.init_node('big_hero', anonymous=True)
      # subscriber for getting info from the board
    rospy.Subscriber("/rfid_board_scanner/board_ids", BoardIds, self.get_electro_board_callback)

    self.current_board_state = ()
    self.electro_board = list()
    self.electro_board_with_tokens = dict()
    self.previous_board_state = dict()
    self.current_board_state = dict()

    #getting the ids from the rfids sensors
    self.ids = rospy.get_param('/rfid_id')
    self.ids = [ int(self.ids[i]) for i in range(len(self.ids))]

    #get the numbers for playing the game
    self.tokens = rospy.get_param('tokens_number')
    #coupling ids with numbers
    self.electro_board_from_ids_to_tokens = self.convert_electro_board_ids_to_tokens(self.ids, self.tokens)


    initial_board_from_launch = rospy.get_param('/initial_board')
    solution_board_from_launch = rospy.get_param('/solution_board')

    self.initial_board = {int(k): str(v) for k, v in initial_board_from_launch.items()}
    self.solution_board = {int(k): str(v) for k, v in solution_board_from_launch.items()}

  def get_tokens_id(self):
    return self.tokens

  def get_initial_board(self):
    return self.initial_board

  def get_solution_board(self):
    return self.solution_board

  #get value from the topic
  def get_electro_board_ids(self):
    return self.electro_board

  #topic subscriber
  def get_electro_board_callback(self, msg):
    self.electro_board = (msg.id)

  def get_electro_board(self):
    return self.get_tokens_by_ids(self.electro_board_from_ids_to_tokens, self.electro_board)

  def convert_electro_board_ids_to_tokens(self, ids, tokens):
    '''
    :param ids:
    :param tokens:
    :return: this method creates a dic where the ids are the keys and the values are the tokens
    '''
    from_ids_to_tokens = dict()
    for i in range(len(ids)):
      from_ids_to_tokens[ids[i]] = tokens[i]
    return from_ids_to_tokens


  def get_tokens_by_ids(self, from_ids_to_tokens_dict, msg):
    '''
    :param electro_board:
    :param msg:
    :return: given a dictionary where key is the id and value is the token change the msg according (0 empty !0 a token id)
    '''
    for i in range(len(msg)):
        if msg[i] == 0:
          # i+1 because the id start
          self.electro_board_with_tokens[i+1] = '0'
        else:
          self.electro_board_with_tokens[i+1] = from_ids_to_tokens_dict.get(msg[i])
    return self.electro_board_with_tokens

  def get_token_location_from_board_state(self, previous_board, token):
    '''
    :param: token
    :return: the location of the token given as input
    '''
    for key, val in previous_board.items():
      if val == token:
        return key


  def detect_move(self, old_board_state, new_board_state):
    movements = list()
    new_location = 0
    for x1 in old_board_state.keys():
        z = new_board_state.get(x1) == old_board_state.get(x1)
        if not z:
            #print('location', x1)
            if (new_board_state.get(x1))!='0':
              token = (new_board_state.get(x1))
            #check where the token is in the new configuration
              prev_location = self.get_token_location_from_board_state(old_board_state, token)
              for key, value in new_board_state.items():
                if token == value and prev_location !=key:
                  new_location = key
              #print("token ", token, " previous location ", prev_location, "new location ", new_location)
              movements.append((token, prev_location, new_location))
    return movements

  def get_current_board(self):
    return self.current_board_state

  def update_board(self, token_id, location):
    '''
    :param token_id: the number
    :param location: its location
    :return:
    '''
    if type(token_id)!=str:
      token_id = str(token_id)


    for loc, token in self.current_board.items():
      if token == (token_id):
        self.current_board[loc] = '0'

    for key in self.current_board.keys():
      if location == key:
        self.current_board[key] = (token_id)
      else:
        pass









def main():


  m = GameFramework()
  initial_board = m.get_initial_board()
  current_board = initial_board.copy()
  ###############################################################

  pub = rospy.Publisher('board_status', StrList, queue_size=10)
  rospy.init_node('big_hero', anonymous=True)
  rate = rospy.Rate(10)  # 10hz
  while not rospy.is_shutdown():
    detected_board = m.get_electro_board()
    detected_movement = m.detect_move(current_board, detected_board)

    if (len(detected_movement)) >= 1 and detected_movement[0][0] != None:
      board_state_msg = detected_board.values()
      pub.publish(board_state_msg)
      rate.sleep()
      current_board = detected_board.copy()
      print("Movement detected")
    else:
      print("No movement detected")

    # a.data = [3250, 2682, 6832, 2296, 8865, 7796, 6955, 8236]
    # pub.publish(a)
    # rate.sleep()
  # while(True):
  #
  #   a = IntList()
  #   a.data = [3250, 2682, 6832, 2296, 8865, 7796, 6955, 8236]
  #   pub.publish(a)
  #   detected_board = m.get_electro_board()
  #   detected_movement = m.detect_move(current_board, detected_board)
  #
  #   if (len(detected_movement)) >= 1 and detected_movement[0][0] != None:
  #     print("Movement detected")
  #     break
  #   else:
  #     print("No movement detected")



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass







