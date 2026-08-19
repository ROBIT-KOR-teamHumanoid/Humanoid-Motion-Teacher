/*
 * Copyright 2024 Myeong Jin Lee
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MAINWINDOW_HPP_
#define MAINWINDOW_HPP_

#include <dirent.h>
#include <yaml-cpp/yaml.h>

#include <QAction>
#include <QColor>
#include <QFileDialog>
#include <QMainWindow>
#include <QMessageBox>
#include <QTimer>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <list>
#include <string>

#include "dynamixel_ctrl.hpp"
#include "dynamixel_init.hpp"

struct Select {
  int id;
  int number;
  bool plus;
  bool reverse;
};

struct Dynamixel_postition {
  uint8_t id;
  double position;
};

struct Step {
  int time;
  int delay;
  int repeat;
  std::string name;
  std::vector<Dynamixel_postition> dxl;
  bool selected;
};

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

 private:
  std::vector<std::string> get_serial_devices();
  void load_motor_config();
  void init_dynamixel_ctrl();

  bool validate_range_settings();
  void setup_ui_for_search();
  bool init_dynamixel_sdk();
  bool search_dynamixels();
  void update_search_progress(uint8_t id, int progress);
  void store_dynamixel_info(uint8_t id);
  bool setup_dynamixel_control();
  bool complete_connection();
  void cleanup_search();
  bool is_dynamixel_ready() const;
  void handle_communication_error(const QString &message);
  bool validate_step_number(int step_number) const;

  void setup_dynamixel_table();

  void init_dynamixel_buttons();

  void oper_dynamixel();
  void oper_dynamixel_V(std::list<Step>::iterator it);
  void oper_motion();

  int search_current_step();

 private Q_SLOTS:
  void on_connect_clicked();
  void on_dynamixel_pos_ctrl_valueChanged(int value);
  void on_motor_set_clicked();
  void on_motor_set_step_clicked();
  void dynamixel_status_timeout();
  void dynamixel_selected(int id);
  void dynamixel_selected_R(int id);
  void on_reset_button_clicked();
  void on_save_step_clicked();
  void on_delete_step_clicked();
  void on_select_step_clicked();
  void on_change_step_clicked();

  void on_buff_save_clicked();
  void on_buff_load_clicked();

  void on_oper_before_clicked();
  void on_oper_next_clicked();

  void on_step_init_clicked();
  void on_save_step_init_clicked();

  void on_motion_operate_clicked();

  void saveMotionToYaml(const std::string &fileName,
                        const std::list<Step> &steps, bool motion);
  void loadMotionFromYaml(const std::string &fileName);

 private slots:
  void saveMotionFile();
  void saveMotionFileas();
  void saveStepFile();
  void saveStepFileas();
  void loadFile();

 private:
  Ui::MainWindow *ui;
  humanoid::DynamixelInit *dynamixel_init_ = nullptr;
  humanoid::DynamixelCtrl *dynamixel_ctrl_ = nullptr;

  bool is_connected = false;
  int motor_baud_rate_ = DEFAULT_BAUDRATE;
  std::string configured_port_ = "/dev/ttyUSB-U2D2";
  std::vector<uint8_t> dynamixel_ids_;
  std::vector<humanoid::DynamixelType> dynamixel_types_;
  std::vector<double> max_position_limit_;
  std::vector<double> min_position_limit_;

  QTimer *dynamixel_status_timer_ = nullptr;
  bool communication_error_shown_ = false;

  std::vector<QLabel *> dynamixel_labels_;
  std::vector<QPushButton *> dynamixel_buttons_;
  std::vector<QPushButton *> dynamixel_buttons_R_;

  std::vector<Select> selected_dynamixels;

  std::vector<double> dynamixel_goal_positions_;
  std::vector<double> dynamixel_goal_velocities_;
  std::vector<double> dynamixel_goal_accelerations_;

  std::vector<double> dynamixel_goal_positions_buff_;

  Step *current_step = nullptr;
  std::vector<Dynamixel_postition> dxl_positions;

  std::list<Step> steps;
  std::unique_ptr<Step> init_step;

  std::string file_path = "../Workspace/";

  int before_value = 0;
};

#endif  // MAINWINDOW_HPP_
