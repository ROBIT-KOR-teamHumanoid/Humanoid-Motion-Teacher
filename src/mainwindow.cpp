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

#include "../include/Humanoid-Motion-Teacher/mainwindow.hpp"

#include "ui_mainwindow.h"

#define RAD2DEG(rad) ((rad) * 180.0 / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.0)

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  QIcon icon(":/image/images/icon.png");
  setWindowIcon(icon);

  for (const auto &port : get_serial_devices()) {
    ui->serialdevice->addItem(QString::fromStdString(port));
  }

  const int preferred_port_index =
      ui->serialdevice->findText("/dev/ttyUSB-U2D2");
  if (preferred_port_index >= 0) {
    ui->serialdevice->setCurrentIndex(preferred_port_index);
  }

  ui->min_range->setValue(0);
  ui->max_range->setValue(6);
  ui->min_range_2->setValue(10);
  ui->max_range_2->setValue(23);

  setup_dynamixel_table();
  init_dynamixel_buttons();

  dynamixel_status_timer_ = new QTimer(this);
  connect(dynamixel_status_timer_, SIGNAL(timeout()), this,
          SLOT(dynamixel_status_timeout()));

  for (size_t i = 0; i < dynamixel_buttons_.size(); ++i) {
    connect(dynamixel_buttons_[i], &QPushButton::clicked, this,
            [this, i]() { dynamixel_selected(i); });
  }

  for (size_t i = 0; i < dynamixel_buttons_.size(); ++i) {
    connect(dynamixel_buttons_R_[i], &QPushButton::clicked, this,
            [this, i]() { dynamixel_selected_R(i); });
  }

  connect(ui->actionSaveMotionFile, &QAction::triggered, this,
          &MainWindow::saveMotionFile);
  connect(ui->actionSaveMotionFileas, &QAction::triggered, this,
          &MainWindow::saveMotionFileas);
  connect(ui->actionSaveStepFile, &QAction::triggered, this,
          &MainWindow::saveStepFile);
  connect(ui->actionSaveStepFileas, &QAction::triggered, this,
          &MainWindow::saveStepFileas);

  connect(ui->actionOpen_Motion_file, &QAction::triggered, this,
          &MainWindow::loadFile);
  connect(ui->actionOpen_Step_file, &QAction::triggered, this,
          &MainWindow::loadFile);
  ui->file_path->setText(QString::fromStdString(file_path));

  init_step = std::make_unique<Step>();
}

MainWindow::~MainWindow() {
  delete dynamixel_status_timer_;
  delete dynamixel_init_;
  delete dynamixel_ctrl_;
  delete ui;
}

std::vector<std::string> MainWindow::get_serial_devices() {
  std::vector<std::string> available_ports;

  const std::filesystem::path dev_path("/dev");

  try {
    for (const auto &entry : std::filesystem::directory_iterator(dev_path)) {
      std::string filename = entry.path().filename().string();

      if (filename.find("ttyUSB") != std::string::npos ||
          filename.find("ttyACM") != std::string::npos) {
        available_ports.push_back("/dev/" + filename);
      }
    }
  } catch (const std::filesystem::filesystem_error &e) {
  }

  return available_ports;
}

void MainWindow::init_dynamixel_ctrl() {
  if (!validate_range_settings()) {
    std::cout << "Invalid range settings" << std::endl;
    return;
  } else {
    std::cout << "Valid range settings" << std::endl;
  }

  setup_ui_for_search();

  if (!init_dynamixel_sdk()) {
    std::cout << "Failed to initialize Dynamixel SDK" << std::endl;
    return;
  } else {
    std::cout << "Initialized Dynamixel SDK" << std::endl;
  }

  if (!search_dynamixels()) {
    std::cout << "Failed to search Dynamixel" << std::endl;
    return;
  } else {
    std::cout << "Searched Dynamixel" << std::endl;
  }

  if (!setup_dynamixel_control()) {
    std::cout << "Failed to setup Dynamixel control" << std::endl;
    return;
  } else {
    std::cout << "Setup Dynamixel control" << std::endl;
  }

  if (!complete_connection()) {
    cleanup_search();
  }
}

bool MainWindow::validate_range_settings() {
  if (ui->min_range->value() > ui->max_range->value()) {
    QMessageBox::critical(this, "Error",
                          "Range 1 start ID is greater than its end ID");
    return false;
  }
  if (ui->min_range_2->value() > ui->max_range_2->value()) {
    QMessageBox::critical(this, "Error",
                          "Range 2 start ID is greater than its end ID");
    return false;
  }
  return true;
}

void MainWindow::setup_ui_for_search() {
  ui->connect->setEnabled(false);
  ui->connect->setText("Searching...");
  ui->connect->setStyleSheet("color: white;");

  const int first_range_size =
      ui->max_range->value() - ui->min_range->value() + 1;
  const int second_range_size =
      ui->max_range_2->value() - ui->min_range_2->value() + 1;
  ui->search_process->setRange(0, first_range_size + second_range_size);
  ui->search_process->setValue(0);
  ui->search_process->setVisible(true);
}

bool MainWindow::init_dynamixel_sdk() {
  if (dynamixel_init_ != nullptr) {
    delete dynamixel_init_;
    dynamixel_init_ = nullptr;
  }

  dynamixel_init_ = new humanoid::DynamixelInit(
      ui->serialdevice->currentText().toStdString());

  try {
    dynamixel_init_->init_sdk();
    return true;
  } catch (const std::runtime_error &e) {
    QMessageBox::critical(this, "Error", e.what());
    delete dynamixel_init_;
    dynamixel_init_ = nullptr;
    ui->connect->setEnabled(true);
    return false;
  }
}

bool MainWindow::search_dynamixels() {
  dynamixel_ids_.clear();
  dynamixel_types_.clear();
  max_position_limit_.clear();
  min_position_limit_.clear();
  dynamixel_goal_positions_.clear();
  dynamixel_goal_velocities_.clear();
  dynamixel_goal_accelerations_.clear();

  std::vector<uint8_t> configured_ids;
  const auto append_range = [&configured_ids](int first, int last) {
    for (int id = first; id <= last; ++id) {
      const auto value = static_cast<uint8_t>(id);
      if (std::find(configured_ids.begin(), configured_ids.end(), value) ==
          configured_ids.end()) {
        configured_ids.push_back(value);
      }
    }
  };

  // Both end IDs are inclusive. Overlapping ranges are searched only once.
  append_range(ui->min_range->value(), ui->max_range->value());
  append_range(ui->min_range_2->value(), ui->max_range_2->value());

  int progress = 0;
  ui->search_process->setRange(0, static_cast<int>(configured_ids.size()));
  for (const uint8_t id : configured_ids) {
    update_search_progress(id, ++progress);
    if (dynamixel_init_->ping_dynamixel(id)) {
      store_dynamixel_info(id);
    }
  }

  if (dynamixel_ids_.empty()) {
    QMessageBox::critical(this, "Error", "No Dynamixel found");
    cleanup_search();
    return false;
  }

  QMessageBox::information(
      this, "Success",
      "Found " + QString::number(dynamixel_ids_.size()) + " devices");

  dynamixel_goal_positions_.resize(dynamixel_ids_.size(), 0);
  dynamixel_goal_velocities_.resize(dynamixel_ids_.size(), 0);
  dynamixel_goal_accelerations_.resize(dynamixel_ids_.size(), 0);
  return true;
}

void MainWindow::update_search_progress(uint8_t id, int progress) {
  ui->search_process->setValue(progress);
  ui->search_stat->setText("Searching ID: " +
                           QString::number(static_cast<int>(id)));
  QApplication::processEvents();
}

void MainWindow::store_dynamixel_info(uint8_t id) {
  dynamixel_ids_.push_back(id);
  dynamixel_labels_[id]->setStyleSheet("color: green;");
  dynamixel_types_.push_back(humanoid::DynamixelType::MX);
  max_position_limit_.push_back(M_PI);
  min_position_limit_.push_back(-M_PI);
}

bool MainWindow::setup_dynamixel_control() {
  try {
    // Release the discovery connection before opening the same serial device
    // for normal motor control.
    delete dynamixel_init_;
    dynamixel_init_ = nullptr;

    delete dynamixel_ctrl_;
    dynamixel_ctrl_ = nullptr;
    dynamixel_ctrl_ = new humanoid::DynamixelCtrl(
        ui->serialdevice->currentText().toStdString(), DEFAULT_BAUDRATE, dynamixel_ids_,
        dynamixel_types_, max_position_limit_, min_position_limit_);
    dynamixel_goal_positions_.resize(dynamixel_ids_.size(), 0);
    dynamixel_goal_velocities_.resize(dynamixel_ids_.size(), 0);
    dynamixel_goal_accelerations_.resize(dynamixel_ids_.size(), 0);
    return true;
  } catch (const std::runtime_error &e) {
    QMessageBox::critical(this, "Error", e.what());
    delete dynamixel_ctrl_;
    dynamixel_ctrl_ = nullptr;
    is_connected = false;
    return false;
  }
}

bool MainWindow::complete_connection() {
  try {
    dynamixel_ctrl_->set_torque(true);
  } catch (const std::exception &e) {
    QMessageBox::critical(
        this, tr("Connection failed"),
        tr("Failed to enable Dynamixel torque.\n%1").arg(e.what()));
    is_connected = false;
    ui->connect->setEnabled(true);
    ui->connect->setText("Connect");
    ui->serialdevice->setEnabled(true);
    return false;
  }

  is_connected = true;
  communication_error_shown_ = false;
  ui->connect->setStyleSheet("color: green;");
  ui->connect->setText("Disconnect");
  ui->serialdevice->setDisabled(true);
  ui->connect->setEnabled(true);
  dynamixel_status_timer_->start(100);
  return true;
}

void MainWindow::cleanup_search() {
  delete dynamixel_init_;
  dynamixel_init_ = nullptr;
  ui->connect->setEnabled(true);
}

bool MainWindow::is_dynamixel_ready() const {
  return is_connected && dynamixel_ctrl_ != nullptr;
}

void MainWindow::handle_communication_error(const QString &message) {
  is_connected = false;
  dynamixel_status_timer_->stop();
  ui->connect->setText("Reconnect");
  ui->connect->setStyleSheet("color: red;");
  ui->serialdevice->setEnabled(true);

  if (communication_error_shown_) {
    return;
  }

  communication_error_shown_ = true;
  QMessageBox::critical(
      this, tr("Dynamixel connection lost"),
      tr("Dynamixel communication failed. Motor commands have been stopped.\n"
         "The motion can still be saved.\n\n%1")
          .arg(message));
}

bool MainWindow::validate_step_number(int step_number) const {
  return step_number >= 1 &&
         step_number <= static_cast<int>(steps.size());
}

void MainWindow::setup_dynamixel_table() {
  QStringList header;
  header << "ID"
         << "Torque"
         << "Error"
         << "Position"
         << "Velocity"
         << "Current"
         << "Temp"
         << "Voltage";

  ui->dynamixel->setColumnCount(header.size());
  ui->dynamixel->setHorizontalHeaderLabels(header);
  ui->dynamixel->setEditTriggers(QAbstractItemView::NoEditTriggers);
  ui->dynamixel->horizontalHeader()->setSectionResizeMode(
      QHeaderView::ResizeToContents);
  ui->dynamixel->setWordWrap(true);
  for (int i = 0; i < ui->dynamixel->columnCount(); ++i) {
    ui->dynamixel->horizontalHeader()->setMinimumSectionSize(50);
    ui->dynamixel->horizontalHeader()->setMaximumSectionSize(150);
  }
  ui->dynamixel->horizontalHeader()->setStyleSheet(
      "QHeaderView::section { "
      "  background-color: #404040; "
      "  color: white; "
      "  padding: 2px; "
      "  border: 1px solid #303030; "
      "  font-weight: bold; "
      "}");

  QStringList header2;
  header2 << "ID";

  ui->selected_id->setColumnCount(header2.size());
  ui->selected_id->setHorizontalHeaderLabels(header2);
  ui->selected_id->setEditTriggers(QAbstractItemView::NoEditTriggers);
  ui->selected_id->horizontalHeader()->setSectionResizeMode(
      QHeaderView::ResizeToContents);
  ui->selected_id->setWordWrap(true);
  for (int i = 0; i < ui->selected_id->columnCount(); ++i) {
    ui->selected_id->horizontalHeader()->setMinimumSectionSize(50);
    ui->selected_id->horizontalHeader()->setMaximumSectionSize(150);
  }
  ui->selected_id->horizontalHeader()->setStyleSheet(
      "QHeaderView::section { "
      "  background-color: #404040; "
      "  color: white; "
      "  padding: 2px; "
      "  border: 1px solid #303030; "
      "  font-weight: bold; "
      "}");

  QStringList header3;
  header3 << "Name"
          << "Time"
          << "Delay"
          << "Repeat";

  ui->dynamixel_step_data->setColumnCount(header3.size());
  ui->dynamixel_step_data->setHorizontalHeaderLabels(header3);
  ui->dynamixel_step_data->setEditTriggers(QAbstractItemView::NoEditTriggers);
  ui->dynamixel_step_data->horizontalHeader()->setSectionResizeMode(
      QHeaderView::ResizeToContents);
  ui->dynamixel_step_data->setWordWrap(true);
  for (int i = 0; i < ui->dynamixel_step_data->columnCount(); ++i) {
    ui->dynamixel_step_data->horizontalHeader()->setMinimumSectionSize(50);
    ui->dynamixel_step_data->horizontalHeader()->setMaximumSectionSize(150);
  }
  ui->dynamixel_step_data->horizontalHeader()->setStyleSheet(
      "QHeaderView::section { "
      "  background-color: #404040; "
      "  color: white; "
      "  padding: 2px; "
      "  border: 1px solid #303030; "
      "  font-weight: bold; "
      "}");
}

void MainWindow::init_dynamixel_buttons() {
  dynamixel_buttons_.clear();
  dynamixel_labels_.clear();

  dynamixel_labels_.push_back(ui->motor0_label);
  dynamixel_buttons_.push_back(ui->motor0_sel);
  dynamixel_buttons_R_.push_back(ui->motor0_sel_R);

  dynamixel_labels_.push_back(ui->motor1_label);
  dynamixel_buttons_.push_back(ui->motor1_sel);
  dynamixel_buttons_R_.push_back(ui->motor1_sel_R);

  dynamixel_labels_.push_back(ui->motor2_label);
  dynamixel_buttons_.push_back(ui->motor2_sel);
  dynamixel_buttons_R_.push_back(ui->motor2_sel_R);

  dynamixel_labels_.push_back(ui->motor3_label);
  dynamixel_buttons_.push_back(ui->motor3_sel);
  dynamixel_buttons_R_.push_back(ui->motor3_sel_R);

  dynamixel_labels_.push_back(ui->motor4_label);
  dynamixel_buttons_.push_back(ui->motor4_sel);
  dynamixel_buttons_R_.push_back(ui->motor4_sel_R);

  dynamixel_labels_.push_back(ui->motor5_label);
  dynamixel_buttons_.push_back(ui->motor5_sel);
  dynamixel_buttons_R_.push_back(ui->motor5_sel_R);

  dynamixel_labels_.push_back(ui->motor6_label);
  dynamixel_buttons_.push_back(ui->motor6_sel);
  dynamixel_buttons_R_.push_back(ui->motor6_sel_R);

  dynamixel_labels_.push_back(ui->motor7_label);
  dynamixel_buttons_.push_back(ui->motor7_sel);
  dynamixel_buttons_R_.push_back(ui->motor7_sel_R);

  dynamixel_labels_.push_back(ui->motor8_label);
  dynamixel_buttons_.push_back(ui->motor8_sel);
  dynamixel_buttons_R_.push_back(ui->motor8_sel_R);

  dynamixel_labels_.push_back(ui->motor9_label);
  dynamixel_buttons_.push_back(ui->motor9_sel);
  dynamixel_buttons_R_.push_back(ui->motor9_sel_R);

  dynamixel_labels_.push_back(ui->motor10_label);
  dynamixel_buttons_.push_back(ui->motor10_sel);
  dynamixel_buttons_R_.push_back(ui->motor10_sel_R);

  dynamixel_labels_.push_back(ui->motor11_label);
  dynamixel_buttons_.push_back(ui->motor11_sel);
  dynamixel_buttons_R_.push_back(ui->motor11_sel_R);

  dynamixel_labels_.push_back(ui->motor12_label);
  dynamixel_buttons_.push_back(ui->motor12_sel);
  dynamixel_buttons_R_.push_back(ui->motor12_sel_R);

  dynamixel_labels_.push_back(ui->motor13_label);
  dynamixel_buttons_.push_back(ui->motor13_sel);
  dynamixel_buttons_R_.push_back(ui->motor13_sel_R);

  dynamixel_labels_.push_back(ui->motor14_label);
  dynamixel_buttons_.push_back(ui->motor14_sel);
  dynamixel_buttons_R_.push_back(ui->motor14_sel_R);

  dynamixel_labels_.push_back(ui->motor15_label);
  dynamixel_buttons_.push_back(ui->motor15_sel);
  dynamixel_buttons_R_.push_back(ui->motor15_sel_R);

  dynamixel_labels_.push_back(ui->motor16_label);
  dynamixel_buttons_.push_back(ui->motor16_sel);
  dynamixel_buttons_R_.push_back(ui->motor16_sel_R);

  dynamixel_labels_.push_back(ui->motor17_label);
  dynamixel_buttons_.push_back(ui->motor17_sel);
  dynamixel_buttons_R_.push_back(ui->motor17_sel_R);

  dynamixel_labels_.push_back(ui->motor18_label);
  dynamixel_buttons_.push_back(ui->motor18_sel);
  dynamixel_buttons_R_.push_back(ui->motor18_sel_R);

  dynamixel_labels_.push_back(ui->motor19_label);
  dynamixel_buttons_.push_back(ui->motor19_sel);
  dynamixel_buttons_R_.push_back(ui->motor19_sel_R);

  dynamixel_labels_.push_back(ui->motor20_label);
  dynamixel_buttons_.push_back(ui->motor20_sel);
  dynamixel_buttons_R_.push_back(ui->motor20_sel_R);

  dynamixel_labels_.push_back(ui->motor21_label);
  dynamixel_buttons_.push_back(ui->motor21_sel);
  dynamixel_buttons_R_.push_back(ui->motor21_sel_R);

  dynamixel_labels_.push_back(ui->motor22_label);
  dynamixel_buttons_.push_back(ui->motor22_sel);
  dynamixel_buttons_R_.push_back(ui->motor22_sel_R);

  dynamixel_labels_.push_back(ui->motor23_label);
  dynamixel_buttons_.push_back(ui->motor23_sel);
  dynamixel_buttons_R_.push_back(ui->motor23_sel_R);

  dynamixel_labels_.push_back(ui->motor24_label);
  dynamixel_buttons_.push_back(ui->motor24_sel);
  dynamixel_buttons_R_.push_back(ui->motor24_sel_R);

  dynamixel_labels_.push_back(ui->motor25_label);
  dynamixel_buttons_.push_back(ui->motor25_sel);
  dynamixel_buttons_R_.push_back(ui->motor25_sel_R);

  for (auto btn : dynamixel_buttons_) {
    btn->setText("0");
    btn->setStyleSheet("background-color: #FFFFFF; color: black;");
  }

  for (auto btn : dynamixel_buttons_R_) {
    btn->setText("O");
    btn->setStyleSheet("background-color: #FFFF00; color: black;");
  }

  for (auto &label : dynamixel_labels_) {
    label->setStyleSheet("color: red;");
  }
}

void MainWindow::on_connect_clicked() {
  if (is_connected) {
    ui->connect->setStyleSheet("color: red;");
    ui->connect->setText("Connect");
    ui->serialdevice->setDisabled(false);
    ui->search_stat->setText("");
    ui->search_process->setValue(0);
    is_connected = false;
    dynamixel_ids_.clear();
    dynamixel_types_.clear();
    max_position_limit_.clear();
    min_position_limit_.clear();
    delete dynamixel_ctrl_;
    dynamixel_ctrl_ = nullptr;
    dynamixel_status_timer_->stop();
    for (auto &label : dynamixel_labels_) {
      label->setStyleSheet("color: red;");
    }

    for (auto &btn : dynamixel_buttons_) {
      btn->setText("X");
      btn->setStyleSheet("background-color: #FFFFFF; color: black;");
      btn->setDisabled(false);
    }

    for (auto &btn : dynamixel_buttons_R_) {
      btn->setText("X");
      btn->setStyleSheet("background-color: #FFFF00; color: black;");
      btn->setDisabled(false);
    }

    ui->dynamixel->clearContents();
    ui->selected_id->clearContents();
    ui->dynamixel_step_data->clearContents();
    setup_dynamixel_table();
  } else {
    init_dynamixel_ctrl();
    if (!is_dynamixel_ready()) {
      return;
    }
    for (int i = 0; i < dynamixel_buttons_.size(); i++) {
      if (std::find(dynamixel_ids_.begin(), dynamixel_ids_.end(), i) ==
          dynamixel_ids_.end()) {
        dynamixel_buttons_[i]->setEnabled(false);
        dynamixel_buttons_[i]->setStyleSheet(
            "background-color: #9A9C96; color: black;");
        dynamixel_buttons_[i]->setText("X");

        dynamixel_buttons_R_[i]->setEnabled(false);
        dynamixel_buttons_R_[i]->setStyleSheet(
            "background-color: #9A9C96; color: black;");
        dynamixel_buttons_R_[i]->setText("X");
      }
    }
  }
}

void MainWindow::on_dynamixel_pos_ctrl_valueChanged(int value) {
  if (!is_dynamixel_ready()) {
    before_value = value;
    return;
  }

  for (auto &s : selected_dynamixels) {
    if (s.plus) {
      dynamixel_goal_positions_[s.number] +=
          (value - before_value) / 1000.00 * M_PI;
    } else {
      dynamixel_goal_positions_[s.number] -=
          (value - before_value) / 1000.00 * M_PI;
    }
  }

  for (int i = 0; i < dynamixel_types_.size(); i++) {
    int _id = static_cast<int>(dynamixel_ctrl_->dynamixels[i]->get_id());
    std::string value_ =
        std::to_string(static_cast<int>(RAD2DEG(dynamixel_goal_positions_[i])));
    dynamixel_buttons_[_id]->setText(value_.c_str());
  }

  before_value = value;

  oper_dynamixel();
}

void MainWindow::on_motor_set_clicked() {
  oper_dynamixel();
}

void MainWindow::on_motor_set_step_clicked() {}

void MainWindow::dynamixel_status_timeout() {
  // for (int i = 0; i < dynamixel_ids_.size(); i++) {
  //   std::cout << "dynamixel_goal_positions_[" << i
  //             << "]: " << dynamixel_goal_positions_[i] << std::endl;
  // }
  // try {
  //   dynamixel_ctrl_->read_dynamixel_status();
  // } catch (const std::runtime_error &e) {
  //   QMessageBox::critical(this, "Error to read dynamixel status",
  //   e.what()); try {
  //     dynamixel_ctrl_->auto_reboot();
  //   } catch (const std::runtime_error &e) {
  //     QMessageBox::critical(this, "Error to auto reboot", e.what());
  //   }
  // }

  // if (ui->dynamixel->rowCount() != dynamixel_ids_.size()) {
  //   ui->dynamixel->clearContents();
  //   ui->dynamixel->setRowCount(dynamixel_ids_.size());
  // }

  // for (size_t row = 0; row < dynamixel_ids_.size(); row++) {
  //   auto createOrUpdateItem = [this, row](int column, const QString &text)
  //   {
  //     QTableWidgetItem *item = ui->dynamixel->item(row, column);
  //     if (!item) {
  //       item = new QTableWidgetItem(text);
  //       ui->dynamixel->setItem(row, column, item);
  //     } else {
  //       item->setText(text);
  //     }
  //     return item;
  //   };

  //   createOrUpdateItem(0, QString::number(dynamixel_ids_[row]));
  //   auto *torque = createOrUpdateItem(
  //       1,
  //       dynamixel_ctrl_->dynamixels[row]->get_torque_status() ? "ON" :
  //       "OFF");
  //   torque->setForeground(dynamixel_ctrl_->dynamixels[row]->get_torque_status()
  //                             ? Qt::green
  //                             : Qt::red);

  //   auto *error = createOrUpdateItem(
  //       2,
  //       QString::number(dynamixel_ctrl_->dynamixels[row]->get_error_status()));
  //   error->setForeground(
  //       dynamixel_ctrl_->dynamixels[row]->get_error_status() > 0 ? Qt::red
  //                                                                :
  //                                                                Qt::green);

  //   createOrUpdateItem(
  //       3,
  //       QString::number(
  //           dynamixel_ctrl_->dynamixels[row]->get_present_position(), 'f',
  //           2));
  //   createOrUpdateItem(
  //       4,
  //       QString::number(
  //           dynamixel_ctrl_->dynamixels[row]->get_present_velocity(), 'f',
  //           2));
  //   createOrUpdateItem(
  //       5,
  //       QString::number(dynamixel_ctrl_->dynamixels[row]->get_present_current(),
  //                       'f', 2));

  //   auto *temp = createOrUpdateItem(
  //       6, QString::number(
  //              dynamixel_ctrl_->dynamixels[row]->get_present_temperature(),
  //              'f', 2));
  //   temp->setForeground(
  //       [temp =
  //       dynamixel_ctrl_->dynamixels[row]->get_present_temperature()]() {
  //         if (temp >= 25 && temp <= 45) {
  //           return Qt::green;
  //         } else if (temp > 45 && temp <= 55) {
  //           return Qt::yellow;
  //         } else {
  //           return Qt::red;
  //         }
  //       }());

  //   auto *voltage = createOrUpdateItem(
  //       7,
  //       QString::number(dynamixel_ctrl_->dynamixels[row]->get_present_voltage(),
  //                       'f', 2));
  //   double current_voltage =
  //       dynamixel_ctrl_->dynamixels[row]->get_present_voltage();
  //   voltage->setForeground(
  //       (current_voltage >= 10 && current_voltage <= 15) ? Qt::green :
  //       Qt::red);
  // }

  // ui->dynamixel->resizeColumnsToContents();
  // for (int i = 0; i < ui->dynamixel->columnCount(); i++) {
  //   ui->dynamixel->horizontalHeader()->setSectionResizeMode(
  //       i, QHeaderView::Stretch);
  // }

  if (ui->selected_id->rowCount() != selected_dynamixels.size()) {
    ui->selected_id->clearContents();
    ui->selected_id->setRowCount(selected_dynamixels.size());
  }

  for (size_t row = 0; row < selected_dynamixels.size(); row++) {
    auto createOrUpdateItem = [this, row](int column, const QString &text) {
      QTableWidgetItem *item = ui->selected_id->item(row, column);
      if (!item) {
        item = new QTableWidgetItem(text);
        ui->selected_id->setItem(row, column, item);
      } else {
        item->setText(text);
      }
      return item;
    };

    auto *id =
        createOrUpdateItem(0, QString::number(selected_dynamixels[row].id));
    id->setForeground(selected_dynamixels[row].reverse > 0 ? Qt::yellow
                                                           : Qt::white);
  }

  if (ui->dynamixel_step_data->rowCount() != steps.size()) {
    ui->dynamixel_step_data->clearContents();
    ui->dynamixel_step_data->setRowCount(steps.size());
  }

  int index = 0;

  std::list<Step>::iterator it = steps.begin();

  for (size_t row = 0; row < steps.size(); row++) {
    auto createOrUpdateItem = [this, row](int column, const QString &text) {
      QTableWidgetItem *item = ui->dynamixel_step_data->item(row, column);
      if (!item) {
        item = new QTableWidgetItem(text);
        ui->dynamixel_step_data->setItem(row, column, item);
      } else {
        item->setText(text);
      }
      return item;
    };

    QString qName = QString::fromStdString(it->name);

    auto *name = createOrUpdateItem(0, qName);
    name->setForeground(it->selected ? Qt::green : Qt::white);

    auto *time = createOrUpdateItem(1, QString::number(it->time, 'd', 2));
    time->setForeground(it->selected ? Qt::green : Qt::white);

    auto *delay = createOrUpdateItem(2, QString::number(it->delay, 'd', 2));
    delay->setForeground(it->selected ? Qt::green : Qt::white);

    auto *repeat = createOrUpdateItem(3, QString::number(it->repeat, 'd', 2));
    repeat->setForeground(it->selected ? Qt::green : Qt::white);

    it++;
  }
}

void MainWindow::dynamixel_selected(int id) {
  if (!is_dynamixel_ready()) {
    return;
  }

  bool erase = false;
  for (int i = 0; i < selected_dynamixels.size(); i++) {
    if (selected_dynamixels[i].id == id) {
      if (selected_dynamixels[i].reverse) {
        selected_dynamixels.erase(selected_dynamixels.begin() + i);
        erase = true;
      }
    }
  }

  if (erase) {
    for (int i = 0; i < selected_dynamixels.size(); i++) {
      if (selected_dynamixels[i].id == id) {
        return;
      }
    }

    dynamixel_buttons_[id]->setStyleSheet(
        "background-color: #FFFFFF; color: black;");
    dynamixel_buttons_R_[id]->setStyleSheet(
        "background-color: #FFFF00; color: black;");

    return;
  }

  dynamixel_buttons_[id]->setStyleSheet(
      "background-color: #0000FF; color: white;");

  if (!is_connected) {
    std::cout << "Not connected" << std::endl;
    return;
  }

  int numb = -1;

  for (int i = 0; i < dynamixel_ctrl_->dynamixels.size(); i++) {
    if (dynamixel_ctrl_->dynamixels[i]->get_id() == id) {
      numb = i;
      break;
    }
  }

  if (numb == -1) {
    std::cout << "Dynamixel not found" << std::endl;
    return;
  }

  if (id % 2 == 0) {
    Select s;
    s.id = id;
    s.number = numb;
    s.plus = false;
    s.reverse = false;
    selected_dynamixels.push_back(s);
  } else {
    Select s;
    s.id = id;
    s.number = numb;
    s.plus = true;
    s.reverse = false;
    selected_dynamixels.push_back(s);
  }
}

void MainWindow::dynamixel_selected_R(int id) {
  if (!is_dynamixel_ready()) {
    return;
  }

  bool erase = false;

  for (int i = 0; i < selected_dynamixels.size(); i++) {
    if (selected_dynamixels[i].id == id) {
      if (!selected_dynamixels[i].reverse) {
        selected_dynamixels.erase(selected_dynamixels.begin() + i);
        erase = true;
      }
    }
  }

  if (erase) {
    for (int i = 0; i < selected_dynamixels.size(); i++) {
      if (selected_dynamixels[i].id == id) {
        return;
      }
    }

    dynamixel_buttons_[id]->setStyleSheet(
        "background-color: #FFFFFF; color: black;");
    dynamixel_buttons_R_[id]->setStyleSheet(
        "background-color: #FFFF00; color: black;");

    return;
  }

  dynamixel_buttons_R_[id]->setStyleSheet(
      "background-color: #0000FF; color: white;");

  if (!is_connected) {
    std::cout << "Not connected" << std::endl;
    return;
  }

  int numb = -1;

  for (int i = 0; i < dynamixel_ctrl_->dynamixels.size(); i++) {
    if (dynamixel_ctrl_->dynamixels[i]->get_id() == id) {
      numb = i;
      break;
    }
  }

  if (numb == -1) {
    std::cout << "Dynamixel not found" << std::endl;
    return;
  }

  if (id % 2 == 0) {
    Select s;
    s.id = id;
    s.number = numb;
    s.plus = true;
    s.reverse = true;
    selected_dynamixels.push_back(s);
  } else {
    Select s;
    s.id = id;
    s.number = numb;
    s.plus = false;
    s.reverse = true;
    selected_dynamixels.push_back(s);
  }
}

void MainWindow::on_reset_button_clicked() {
  selected_dynamixels.clear();

  if (!is_dynamixel_ready()) {
    return;
  }

  for (int i = 0; i < dynamixel_buttons_.size(); i++) {
    for (int j = 0; j < dynamixel_ctrl_->dynamixels.size(); j++) {
      if (dynamixel_ctrl_->dynamixels[j]->get_id() == i) {
        dynamixel_buttons_[i]->setStyleSheet(
            "background-color: #FFFFFF; color: black;");
        dynamixel_buttons_R_[i]->setStyleSheet(
            "background-color: #FFFF00; color: black;");
      }
    }
  }
}

void MainWindow::on_save_step_clicked() {
  if (!is_dynamixel_ready()) {
    QMessageBox::warning(this, tr("Dynamixel not connected"),
                         tr("Connect Dynamixel before saving a new step."));
    return;
  }

  Step step;
  step.time = ui->step_time->toPlainText().toInt();
  step.delay = ui->step_delay->toPlainText().toInt();
  step.repeat = ui->step_repeat->toPlainText().toInt();
  step.name = ui->step_name->toPlainText().toStdString();
  step.selected = true;

  for (int i = 0; i < dynamixel_ctrl_->dynamixels.size(); i++) {
    Dynamixel_postition dxl;
    dxl.id = dynamixel_ctrl_->dynamixels[i]->get_id();
    dxl.position = dynamixel_goal_positions_[i];
    step.dxl.push_back(dxl);
  }

  for (std::list<Step>::iterator it = steps.begin(); it != steps.end(); it++) {
    it->selected = false;
  }

  steps.push_back(step);

  current_step = &steps.back();
}

void MainWindow::on_delete_step_clicked() {
  if (steps.size() == 0) {
    std::cout << "No step to delete" << std::endl;
    return;
  }

  current_step = nullptr;

  steps.pop_back();

  if (steps.size() == 0) {
    return;
  }
  steps.back().selected = true;
  current_step = &steps.back();
}

void MainWindow::on_select_step_clicked() {
  const int step_number = ui->step_spinbox->value();
  if (!validate_step_number(step_number)) {
    QMessageBox::warning(
        this, tr("Invalid step"),
        tr("Step number must be between 1 and %1.")
            .arg(static_cast<int>(steps.size())));
    return;
  }

  if (!is_dynamixel_ready()) {
    QMessageBox::warning(this, tr("Dynamixel not connected"),
                         tr("Connect Dynamixel before operating a step."));
    return;
  }

  std::list<Step>::iterator it = steps.begin();
  std::advance(it, step_number - 1);

  oper_dynamixel_V(it);

  for (int i = 0; i < dynamixel_ctrl_->dynamixels.size(); i++) {
    for (int j = 0; j < it->dxl.size(); j++) {
      if (dynamixel_ctrl_->dynamixels[i]->get_id() == it->dxl[j].id) {
        dynamixel_goal_positions_[i] = it->dxl[j].position;
        break;
      }
    }
  }

  for (int i = 0; i < dynamixel_ctrl_->dynamixels.size(); i++) {
    int _id = static_cast<int>(dynamixel_ctrl_->dynamixels[i]->get_id());
    std::string value_ =
        std::to_string(static_cast<int>(RAD2DEG(dynamixel_goal_positions_[i])));
    dynamixel_buttons_[_id]->setText(value_.c_str());
  }

  for (std::list<Step>::iterator iter = steps.begin(); iter != steps.end();
       ++iter) {
    if (iter == it) continue;
    iter->selected = false;
  }

  it->selected = true;

  current_step = &(*it);
}

void MainWindow::on_change_step_clicked() {
  const int step_number = ui->step_spinbox->value();
  if (!validate_step_number(step_number)) {
    QMessageBox::warning(
        this, tr("Invalid step"),
        tr("Step number must be between 1 and %1.")
            .arg(static_cast<int>(steps.size())));
    return;
  }

  if (!is_dynamixel_ready()) {
    QMessageBox::warning(this, tr("Dynamixel not connected"),
                         tr("Connect Dynamixel before changing a step."));
    return;
  }

  std::list<Step>::iterator it = steps.begin();
  std::advance(it, step_number - 1);

  Step step;
  step.time = ui->step_time->toPlainText().toInt();
  step.delay = ui->step_delay->toPlainText().toInt();
  step.repeat = ui->step_repeat->toPlainText().toInt();
  step.name = ui->step_name->toPlainText().toStdString();
  step.selected = true;

  for (int i = 0; i < dynamixel_ctrl_->dynamixels.size(); i++) {
    Dynamixel_postition dxl;
    dxl.id = dynamixel_ctrl_->dynamixels[i]->get_id();
    dxl.position = dynamixel_goal_positions_[i];
    step.dxl.push_back(dxl);
  }

  for (std::list<Step>::iterator iter = steps.begin(); iter != steps.end();
       ++iter) {
    if (iter == it) continue;
    iter->selected = false;
  }

  it = steps.erase(it);
  steps.insert(it, step);

  it--;
  current_step = &(*it);
}

void MainWindow::on_buff_save_clicked() {
  dynamixel_goal_positions_buff_ = dynamixel_goal_positions_;
  for (int i = 0; i < dynamixel_goal_positions_buff_.size(); i++) {
    std::cout << "dynamixel_goal_positions_buff_[" << i
              << "]: " << dynamixel_goal_positions_buff_[i] << std::endl;
  }
}

void MainWindow::on_buff_load_clicked() {
  if (!is_dynamixel_ready() ||
      dynamixel_goal_positions_buff_.size() !=
          dynamixel_goal_positions_.size()) {
    QMessageBox::warning(this, tr("Invalid buffered position"),
                         tr("No compatible buffered motor position exists."));
    return;
  }

  dynamixel_goal_positions_ = dynamixel_goal_positions_buff_;

  oper_dynamixel();

  for (int i = 0; i < dynamixel_types_.size(); i++) {
    int _id = static_cast<int>(dynamixel_ctrl_->dynamixels[i]->get_id());
    std::string value_ =
        std::to_string(static_cast<int>(RAD2DEG(dynamixel_goal_positions_[i])));
    dynamixel_buttons_[_id]->setText(value_.c_str());
  }
}

void MainWindow::oper_dynamixel() {
  if (!is_dynamixel_ready()) {
    return;
  }

  try {
    dynamixel_ctrl_->sync_write(dynamixel_goal_positions_,
                                dynamixel_goal_velocities_,
                                dynamixel_goal_accelerations_);
  } catch (const std::exception &e) {
    handle_communication_error(QString::fromUtf8(e.what()));
  }
}

void MainWindow::oper_dynamixel_V(std::list<Step>::iterator it) {
  if (!is_dynamixel_ready() || it == steps.end()) {
    return;
  }

  if (it->time <= 0) {
    QMessageBox::warning(this, tr("Invalid step time"),
                         tr("Step time must be greater than 0 ms."));
    return;
  }

  for (int i = 0; i < dynamixel_ctrl_->dynamixels.size(); i++) {
    for (int j = 0; j < it->dxl.size(); j++) {
      if (dynamixel_ctrl_->dynamixels[i]->get_id() == it->dxl[j].id) {
        dynamixel_goal_velocities_[i] =
            (fabs(dynamixel_goal_positions_[i] - it->dxl[j].position)) /
            (it->time * 0.001);
        break;
      }
    }
  }

  for (int i = 0; i < dynamixel_ctrl_->dynamixels.size(); i++) {
    for (int j = 0; j < it->dxl.size(); j++) {
      if (dynamixel_ctrl_->dynamixels[i]->get_id() == it->dxl[j].id) {
        dynamixel_goal_positions_[i] = it->dxl[j].position;
        break;
      }
    }
  }

  current_step = &(*it);

  oper_dynamixel();

  for (int i = 0; i < dynamixel_goal_velocities_.size(); i++) {
    std::cout << "dynamixel_goal_velocities_[" << i
              << "]: " << dynamixel_goal_velocities_[i] << std::endl;
    dynamixel_goal_velocities_[i] = 0;
  }
}

int MainWindow::search_current_step() {
  int index = -1;
  std::list<Step>::iterator it = steps.begin();

  for (int i = 0; i < steps.size(); i++) {
    if (it->selected) {
      index = i;
      break;
    }
    it++;
  }

  return index;
}

void MainWindow::on_oper_next_clicked() {
  int index = search_current_step();

  if (index == -1) {
    std::cout << "No step selected" << std::endl;
    return;
  } else if (index == steps.size() - 1) {
    std::cout << "Last step selected" << std::endl;
    return;
  }

  std::list<Step>::iterator it = steps.begin();

  for (int i = 0; i < index; i++) {
    it++;
  }
  it++;

  it->selected = true;

  oper_dynamixel_V(it);

  it--;

  it->selected = false;
}

void MainWindow::on_oper_before_clicked() {
  int index = search_current_step();

  if (index == -1) {
    std::cout << "No step selected" << std::endl;
    return;
  } else if (index == 0) {
    std::cout << "First step selected" << std::endl;
    return;
  }

  std::list<Step>::iterator it = steps.begin();

  for (int i = 0; i < index; i++) {
    it++;
  }
  it--;

  it->selected = true;

  oper_dynamixel_V(it);

  it++;

  it->selected = false;
}

void MainWindow::on_step_init_clicked() {
  if (init_step == nullptr) {
    std::cout << "No init step" << std::endl;
    return;
  }

  if (!is_dynamixel_ready()) {
    QMessageBox::warning(this, tr("Dynamixel not connected"),
                         tr("Connect Dynamixel before operating init step."));
    return;
  }

  if (init_step->time <= 0) {
    QMessageBox::warning(this, tr("Invalid step time"),
                         tr("Init step time must be greater than 0 ms."));
    return;
  }

  Step step;
  step.time = init_step->time;
  step.delay = init_step->delay;
  step.repeat = init_step->repeat;
  step.name = init_step->name;
  step.selected = true;

  for (int i = 0; i < init_step->dxl.size(); i++) {
    Dynamixel_postition dxl;
    dxl.id = init_step->dxl[i].id;
    dxl.position = init_step->dxl[i].position;
    step.dxl.push_back(dxl);
  }

  for (int i = 0; i < dynamixel_ctrl_->dynamixels.size(); i++) {
    for (int j = 0; j < init_step->dxl.size(); j++) {
      if (dynamixel_ctrl_->dynamixels[i]->get_id() == init_step->dxl[j].id) {
        dynamixel_goal_velocities_[i] =
            (fabs(dynamixel_goal_positions_[i] - init_step->dxl[j].position)) /
            (init_step->time * 0.001);
        break;
      }
    }
  }

  for (int i = 0; i < dynamixel_ctrl_->dynamixels.size(); i++) {
    for (int j = 0; j < init_step->dxl.size(); j++) {
      if (dynamixel_ctrl_->dynamixels[i]->get_id() == init_step->dxl[j].id) {
        dynamixel_goal_positions_[i] = init_step->dxl[j].position;
        break;
      }
    }
  }

  oper_dynamixel();

  for (int i = 0; i < dynamixel_goal_velocities_.size(); i++) {
    dynamixel_goal_velocities_[i] = 0;
  }

  for (std::list<Step>::iterator iter = steps.begin(); iter != steps.end();
       ++iter) {
    iter->selected = false;
  }

  steps.push_back(step);

  current_step = &steps.back();
}

void MainWindow::on_save_step_init_clicked() {
  if (!is_dynamixel_ready()) {
    QMessageBox::warning(this, tr("Dynamixel not connected"),
                         tr("Connect Dynamixel before saving init step."));
    return;
  }

  init_step.reset(new Step);

  init_step->time = ui->step_time->toPlainText().toInt();
  init_step->delay = ui->step_delay->toPlainText().toInt();
  init_step->repeat = ui->step_repeat->toPlainText().toInt();
  init_step->name = ui->step_name->toPlainText().toStdString();

  for (int i = 0; i < dynamixel_ctrl_->dynamixels.size(); i++) {
    Dynamixel_postition dxl;
    dxl.id = dynamixel_ctrl_->dynamixels[i]->get_id();
    dxl.position = dynamixel_goal_positions_[i];
    init_step->dxl.push_back(dxl);
  }

  std::cout << "Init step saved" << std::endl;
}

void MainWindow::on_motion_operate_clicked() {
  if (steps.size() == 0) {
    std::cout << "No step to operate" << std::endl;
    return;
  }

  if (!is_dynamixel_ready()) {
    QMessageBox::warning(this, tr("Dynamixel not connected"),
                         tr("Connect Dynamixel before operating motion."));
    return;
  }

  oper_motion();
}

void MainWindow::oper_motion() {
  std::list<Step>::iterator it = steps.begin();

  for (int i = 0; i < steps.size(); i++) {
    oper_dynamixel_V(it);
    if (!is_dynamixel_ready()) {
      return;
    }
    std::cout << "time : " << it->time << " delay : " << it->delay
              << " repeat : " << it->repeat << std::endl;

    std::cout << "go time" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(it->time));
    std::cout << "end time" << std::endl;
    std::cout << "go delay" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(it->delay));
    std::cout << "end delay" << std::endl;
    it++;
  }
}

void MainWindow::saveMotionFile() { saveMotionToYaml(file_path, steps, true); }

void MainWindow::saveMotionFileas() {  // 파일 저장 대화상자 표시
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "",
                                                  tr("Files (*.*)"));
  if (fileName.isEmpty()) {
    return;
  }

  saveMotionToYaml(fileName.toStdString(), steps, true);

  file_path = fileName.toStdString();
  ui->file_path->setText(fileName);
  QMessageBox::information(this, tr("File Saved"),
                           tr("File saved successfully: %1").arg(fileName));
}

void MainWindow::saveStepFile() { saveMotionToYaml(file_path, steps, false); }

void MainWindow::saveStepFileas() {
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "",
                                                  tr("Files (*.*)"));
  if (fileName.isEmpty()) {
    return;
  }

  saveMotionToYaml(fileName.toStdString(), steps, false);

  file_path = fileName.toStdString();
  ui->file_path->setText(fileName);
  QMessageBox::information(this, tr("File Saved"),
                           tr("File saved successfully: %1").arg(fileName));
}

void MainWindow::loadFile() {  // 파일 열기 대화상자 표시
  QString fileName = QFileDialog::getOpenFileName(
      this, tr("Open File"), "", tr("YAML Files (*.yaml);;All Files (*)"));
  if (!fileName.isEmpty()) {
    // 선택된 파일을 loadMotionFromYaml 함수로 전달
    try {
      loadMotionFromYaml(fileName.toStdString());
    } catch (const YAML::Exception &e) {
      QMessageBox::critical(this, tr("Load failed"),
                            QString::fromUtf8(e.what()));
    } catch (const std::exception &e) {
      QMessageBox::critical(this, tr("Load failed"),
                            QString::fromUtf8(e.what()));
    }
  }
}

void MainWindow::saveMotionToYaml(const std::string &fileName,
                                  const std::list<Step> &steps, bool motion) {
  if (fileName.empty()) {
    return;
  }

  if (dynamixel_ctrl_ == nullptr) {
    QMessageBox::warning(this, tr("Save failed"),
                         tr("No Dynamixel configuration is available."));
    return;
  }

  if (!motion && current_step == nullptr) {
    QMessageBox::warning(this, tr("Save failed"),
                         tr("No step is selected."));
    return;
  }

  std::string name = ui->motion_name->toPlainText().toStdString();

  YAML::Emitter out;
  out << YAML::BeginSeq;
  out << YAML::BeginMap;
  out << YAML::Key << "name" << YAML::Value << name;
  out << YAML::EndMap;
  out << YAML::BeginMap;
  out << YAML::Key << "motor id" << YAML::Value << YAML::BeginSeq;
  for (int i = 0; i < dynamixel_ctrl_->dynamixels.size(); i++) {
    out << static_cast<int>(dynamixel_ctrl_->dynamixels[i]->get_id());
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
  out << YAML::BeginMap;
  if (motion) {
    out << YAML::Key << "type" << YAML::Value << "motion";
  } else {
    out << YAML::Key << "type" << YAML::Value << "step";
  }
  out << YAML::EndMap;
  if (motion) {
    for (const auto &step : steps) {
      out << YAML::BeginMap;
      out << YAML::Key << "time" << YAML::Value << step.time;
      out << YAML::Key << "delay" << YAML::Value << step.delay;
      out << YAML::Key << "repeat" << YAML::Value << step.repeat;
      out << YAML::Key << "name" << YAML::Value << step.name;
      out << YAML::Key << "selected" << YAML::Value << step.selected;
      out << YAML::Key << "dxl" << YAML::Value << YAML::BeginSeq;
      for (const auto &dxl : step.dxl) {
        out << YAML::BeginMap;
        out << YAML::Key << "id" << YAML::Value << static_cast<int>(dxl.id);
        out << YAML::Key << "position" << YAML::Value << dxl.position;
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  } else {
    out << YAML::BeginMap;
    out << YAML::Key << "time" << YAML::Value << current_step->time;
    out << YAML::Key << "delay" << YAML::Value << current_step->delay;
    out << YAML::Key << "repeat" << YAML::Value << current_step->repeat;
    out << YAML::Key << "name" << YAML::Value << current_step->name;
    out << YAML::Key << "selected" << YAML::Value << false;
    out << YAML::Key << "dxl" << YAML::Value << YAML::BeginSeq;
    for (const auto &dxl : current_step->dxl) {
      out << YAML::BeginMap;
      out << YAML::Key << "id" << YAML::Value << static_cast<int>(dxl.id);
      out << YAML::Key << "position" << YAML::Value << dxl.position;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
  }

  std::ofstream fout(fileName);
  fout << out.c_str();
}

void MainWindow::loadMotionFromYaml(const std::string &fileName) {
  YAML::Node file = YAML::LoadFile(fileName);

  if (!file.IsSequence() || file.size() < 3 || !file[1]["motor id"] ||
      !file[2]["type"]) {
    throw std::runtime_error("Invalid motion YAML format");
  }

  // YAML 파일의 motor id와 현재 활성화된 motor id 비교
  std::vector<int> file_motor_ids = file[1]["motor id"].as<std::vector<int>>();
  std::vector<int> current_motor_ids;
  std::cout << "file_motor_ids.size(): " << file_motor_ids.size() << std::endl;
  if (dynamixel_ctrl_ != nullptr) {
    std::cout << "current_motor_ids.size(): "
              << dynamixel_ctrl_->dynamixels.size() << std::endl;
    for (int i = 0; i < dynamixel_ctrl_->dynamixels.size(); i++) {
      std::cout << "dynamixel_ctrl_->dynamixels[i]->get_id(): "
                << dynamixel_ctrl_->dynamixels[i]->get_id() << std::endl;
      current_motor_ids.push_back(
          static_cast<int>(dynamixel_ctrl_->dynamixels[i]->get_id()));
    }
  }

  if (!current_motor_ids.empty() && file_motor_ids != current_motor_ids) {
    QMessageBox::critical(
        this, tr("Error"),
        tr("호환되지 않는 모션입니다. Dynamixel ID를 확인해주세요."));
    return;
  }

  std::string type = file[2]["type"].as<std::string>();

  if (type == "step") {
    for (std::size_t i = 3; i < file.size(); ++i) {
      Step step;
      step.time = file[i]["time"].as<int>();
      step.delay = file[i]["delay"].as<int>();
      step.repeat = file[i]["repeat"].as<int>();
      step.name = file[i]["name"].as<std::string>();
      step.selected = file[i]["selected"].as<bool>();
      for (const auto &dxl : file[i]["dxl"]) {
        Dynamixel_postition pos;
        pos.id = dxl["id"].as<int>();
        pos.position = dxl["position"].as<double>();
        step.dxl.push_back(pos);
      }
      steps.push_back(step);
    }
  } else if (type == "motion") {
    steps.clear();
    for (std::size_t i = 3; i < file.size(); ++i) {
      Step step;
      step.time = file[i]["time"].as<int>();
      step.delay = file[i]["delay"].as<int>();
      step.repeat = file[i]["repeat"].as<int>();
      step.name = file[i]["name"].as<std::string>();
      step.selected = file[i]["selected"].as<bool>();
      for (const auto &dxl : file[i]["dxl"]) {
        Dynamixel_postition pos;
        pos.id = dxl["id"].as<int>();
        pos.position = dxl["position"].as<double>();
        step.dxl.push_back(pos);
      }
      steps.push_back(step);
    }
  }
}
