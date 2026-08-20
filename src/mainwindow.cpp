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

  ui->min_range->setValue(0);
  ui->max_range->setValue(6);
  ui->min_range_2->setValue(10);
  ui->max_range_2->setValue(23);
  load_motor_config();

  const QString configured_port = QString::fromStdString(configured_port_);
  int preferred_port_index = ui->serialdevice->findText(configured_port);
  if (preferred_port_index < 0 && !configured_port.isEmpty()) {
    ui->serialdevice->addItem(configured_port);
    preferred_port_index = ui->serialdevice->findText(configured_port);
  }
  if (preferred_port_index >= 0) {
    ui->serialdevice->setCurrentIndex(preferred_port_index);
  }

  setup_dynamixel_table();
  init_dynamixel_buttons();

  dynamixel_status_timer_ = new QTimer(this);
  dynamixel_status_timer_->setTimerType(Qt::PreciseTimer);
  connect(dynamixel_status_timer_, SIGNAL(timeout()), this,
          SLOT(dynamixel_status_timeout()));

  temp_load_timer_ = new QTimer(this);
  temp_load_timer_->setTimerType(Qt::PreciseTimer);
  temp_load_timer_->setInterval(10);
  connect(temp_load_timer_, &QTimer::timeout, this, [this]() {
    constexpr int kTempLoadDurationMs = 1000;
    if (!is_dynamixel_ready() ||
        temp_load_start_positions_.size() !=
            temp_load_target_positions_.size()) {
      temp_load_timer_->stop();
      return;
    }
    temp_load_elapsed_ms_ += temp_load_timer_->interval();
    const double ratio = std::min(
        1.0, static_cast<double>(temp_load_elapsed_ms_) /
                 static_cast<double>(kTempLoadDurationMs));
    for (std::size_t i = 0; i < temp_load_start_positions_.size(); ++i) {
      dynamixel_goal_positions_[i] = temp_load_start_positions_[i] +
          (temp_load_target_positions_[i] - temp_load_start_positions_[i]) *
              ratio;
      dynamixel_goal_velocities_[i] = 0.0;
    }
    oper_dynamixel();
    if (ratio >= 1.0) temp_load_timer_->stop();
  });

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
  connect(ui->dynamixel_step_data, &QTableWidget::cellClicked, this,
          [this](int row, int) { select_step_for_edit(row); });

  auto *edit_menu = ui->menuBar->addMenu(tr("Edit"));
  ui->actionUndo->setShortcut(QKeySequence::Undo);
  ui->actionRedo->setShortcuts(
      {QKeySequence::Redo, QKeySequence(Qt::CTRL | Qt::Key_Y)});
  edit_menu->addAction(ui->actionUndo);
  edit_menu->addAction(ui->actionRedo);
  connect(ui->actionUndo, &QAction::triggered, this,
          &MainWindow::undo_step_edit);
  connect(ui->actionRedo, &QAction::triggered, this,
          &MainWindow::redo_step_edit);

  ui->actionSaveMotionFile->setShortcut(QKeySequence::Save);
  recent_motion_menu_ = new QMenu(tr("Recent Motions"), ui->menuFile);
  ui->menuFile->insertMenu(ui->actionRecent_Files, recent_motion_menu_);
  ui->menuFile->removeAction(ui->actionRecent_Files);
  QSettings settings("ROBIT", "HumanoidMotionTeacher");
  recent_motion_files_ = settings.value("recentMotionFiles").toStringList();
  rebuild_recent_motion_menu();

  auto *history_toolbar = addToolBar(tr("History and recent motions"));
  history_toolbar->setObjectName("historyAndRecentMotionsToolBar");
  history_toolbar->setMovable(false);
  history_toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  history_toolbar->addAction(ui->actionUndo);
  history_toolbar->addAction(ui->actionRedo);
  history_toolbar->addSeparator();
  auto *recent_button = new QToolButton(history_toolbar);
  recent_button->setText(tr("Recent Motions"));
  recent_button->setIcon(ui->actionRecent_Files->icon());
  recent_button->setMenu(recent_motion_menu_);
  recent_button->setPopupMode(QToolButton::InstantPopup);
  recent_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  history_toolbar->addWidget(recent_button);
  auto *mirror_action = history_toolbar->addAction(
      style()->standardIcon(QStyle::SP_BrowserReload), tr("Left/Right Mirror"));
  mirror_action->setToolTip(tr("Create or replace mirrored motion steps"));
  connect(mirror_action, &QAction::triggered, this,
          &MainWindow::open_mirror_editor);
  ui->file_path->setText(QString::fromStdString(file_path));

  init_step = std::make_unique<Step>();
}

MainWindow::~MainWindow() {
  if (dynamixel_status_timer_ != nullptr) dynamixel_status_timer_->stop();
  if (temp_load_timer_ != nullptr) temp_load_timer_->stop();
  delete dynamixel_init_;
  delete dynamixel_ctrl_;
  delete dynamixel_status_timer_;
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

void MainWindow::load_motor_config() {
  try {
    const YAML::Node config = YAML::LoadFile(MOTOR_CONFIG_FILE);
    const YAML::Node motor = config["motor"];
    if (!motor || !motor["port"] || !motor["baud_rate"] ||
        !motor["write_hz"] || !motor["read_hz"] ||
        !motor["min_step_time_ms"] || !motor["min_step_delay_ms"] ||
        !motor["undo_history_limit"] ||
        !motor["id_ranges"] || !motor["id_ranges"].IsSequence() ||
        motor["id_ranges"].size() != 2) {
      throw std::runtime_error("Required motor settings are missing");
    }

    const int baud_rate = motor["baud_rate"].as<int>();
    const int write_hz = motor["write_hz"].as<int>();
    const int read_hz = motor["read_hz"].as<int>();
    const int min_step_time_ms = motor["min_step_time_ms"].as<int>();
    const int min_step_delay_ms = motor["min_step_delay_ms"].as<int>();
    const int undo_history_limit = motor["undo_history_limit"].as<int>();
    const int range1_start = motor["id_ranges"][0]["start"].as<int>();
    const int range1_end = motor["id_ranges"][0]["end"].as<int>();
    const int range2_start = motor["id_ranges"][1]["start"].as<int>();
    const int range2_end = motor["id_ranges"][1]["end"].as<int>();
    const auto valid_id = [](int id) { return id >= 0 && id <= 25; };

    if (baud_rate <= 0 || write_hz <= 0 || write_hz > 1000 ||
        read_hz <= 0 || write_hz != read_hz * 10 ||
        min_step_time_ms <= 0 || min_step_delay_ms <= 0 ||
        undo_history_limit <= 0 || undo_history_limit > 50 ||
        !valid_id(range1_start) || !valid_id(range1_end) ||
        !valid_id(range2_start) || !valid_id(range2_end) ||
        range1_start > range1_end || range2_start > range2_end) {
      throw std::runtime_error(
          "Invalid settings: write_hz must be 10 x read_hz (max 1000 Hz), "
          "undo_history_limit must be 1-50, and valid IDs are 0-25");
    }

    configured_port_ = motor["port"].as<std::string>();
    motor_baud_rate_ = baud_rate;
    motor_write_hz_ = write_hz;
    motor_read_hz_ = read_hz;
    min_step_time_ms_ = min_step_time_ms;
    min_step_delay_ms_ = min_step_delay_ms;
    undo_history_limit_ = static_cast<std::size_t>(undo_history_limit);
    mirror_pairs_.clear();
    const YAML::Node pairs = motor["mirror_pairs"];
    if (pairs) {
      if (!pairs.IsSequence()) {
        throw std::runtime_error("mirror_pairs must be a sequence");
      }
      for (const auto& pair : pairs) {
        const int left = pair["left"].as<int>();
        const int right = pair["right"].as<int>();
        const double direction = pair["direction"].as<double>();
        if (!valid_id(left) || !valid_id(right) || left == right ||
            (direction != 1.0 && direction != -1.0)) {
          throw std::runtime_error(
              "mirror pair IDs must be 0-25 and direction must be 1 or -1");
        }
        mirror_pairs_.push_back({static_cast<uint8_t>(left),
                                 static_cast<uint8_t>(right), direction});
      }
    }
    ui->min_range->setValue(range1_start);
    ui->max_range->setValue(range1_end);
    ui->min_range_2->setValue(range2_start);
    ui->max_range_2->setValue(range2_end);
  } catch (const std::exception &e) {
    QMessageBox::warning(
        this, "Motor configuration",
        QString("Could not load %1\n%2\nUsing default motor settings.")
            .arg(MOTOR_CONFIG_FILE, e.what()));
  }
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
      ui->serialdevice->currentText().toStdString(), motor_baud_rate_);

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
  dynamixel_model_numbers_.clear();
  dynamixel_model_names_.clear();
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
    uint16_t model_number = 0;
    if (dynamixel_init_->ping_dynamixel(id, &model_number)) {
      if (!store_dynamixel_info(id, model_number)) {
        cleanup_search();
        return false;
      }
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

bool MainWindow::store_dynamixel_info(uint8_t id, uint16_t model_number) {
  struct ModelInfo { uint16_t number; humanoid::DynamixelType type; const char *name; };
  static const ModelInfo models[] = {
      {30, humanoid::MX28, "MX-28(2.0)"},
      {311, humanoid::MX, "MX-64(2.0)"}, {321, humanoid::MX, "MX-106(2.0)"},
      {1000, humanoid::XH, "XH430-W350"}, {1010, humanoid::XH, "XH430-W210"},
      {1040, humanoid::XH, "XH430-V350"}, {1050, humanoid::XH, "XH430-V210"},
      {1100, humanoid::XH, "XH540-W270"}, {1110, humanoid::XH, "XH540-W150"},
      {1140, humanoid::XH, "XH540-V270"}, {1150, humanoid::XH, "XH540-V150"},
      {2020, humanoid::PH54_200, "PH54-200-S500-R"},
      {2010, humanoid::PH54_100, "PH54-100-S500-R"},
      {2000, humanoid::PH42_020, "PH42-020-S300-R"},
      {2120, humanoid::PM54_060, "PM54-060-S250-R"},
      {2110, humanoid::PM54_040, "PM54-040-S250-R"},
      {2100, humanoid::PM42_010, "PM42-010-S260-R"}};
  const auto found = std::find_if(std::begin(models), std::end(models),
      [model_number](const ModelInfo& model) { return model.number == model_number; });
  if (found == std::end(models)) {
    QMessageBox::critical(this, "Unsupported Dynamixel",
        QString("ID %1: unsupported model number %2")
            .arg(static_cast<int>(id)).arg(model_number));
    return false;
  }
  dynamixel_ids_.push_back(id);
  dynamixel_model_numbers_.push_back(model_number);
  dynamixel_model_names_.push_back(found->name);
  dynamixel_labels_[id]->setStyleSheet("color: green;");
  dynamixel_types_.push_back(found->type);
  max_position_limit_.push_back(M_PI);
  min_position_limit_.push_back(-M_PI);
  return true;
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
        ui->serialdevice->currentText().toStdString(), motor_baud_rate_, dynamixel_ids_,
        dynamixel_model_numbers_, dynamixel_model_names_,
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
    // Never let a freshly-created command buffer (all zeroes) become the
    // first command after reconnect.  Read the real pose and adopt it as the
    // goal for every motor before enabling torque or interactive writes.
    dynamixel_ctrl_->read_dynamixel_status();
    for (std::size_t i = 0; i < dynamixel_ctrl_->dynamixels.size(); ++i) {
      dynamixel_goal_positions_[i] =
          dynamixel_ctrl_->dynamixels[i]->get_present_position();
      dynamixel_goal_velocities_[i] = 0.0;
      dynamixel_goal_accelerations_[i] = 0.0;
      const int id = dynamixel_ctrl_->dynamixels[i]->get_id();
      if (id >= 0 && id < static_cast<int>(dynamixel_buttons_.size())) {
        dynamixel_buttons_[id]->setText(QString::number(
            static_cast<int>(RAD2DEG(dynamixel_goal_positions_[i]))));
      }
    }
    pending_goal_positions_ = dynamixel_goal_positions_;
    pending_goal_velocities_ = dynamixel_goal_velocities_;
    pending_goal_accelerations_ = dynamixel_goal_accelerations_;
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
  motor_command_active_ = false;
  communication_cycle_count_ = 0;
  communication_error_shown_ = false;
  ui->connect->setStyleSheet("color: green;");
  ui->connect->setText("Disconnect");
  ui->serialdevice->setDisabled(true);
  ui->connect->setEnabled(true);
  // All serial I/O is serialized through communication_cycle().  Status is
  // intentionally slower than interactive command writes.
  dynamixel_status_timer_->start(std::max(1, 1000 / motor_write_hz_));
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
  motor_command_active_ = false;
  dynamixel_status_timer_->stop();
  temp_load_timer_->stop();
  ui->connect->setText("Reconnect");
  ui->connect->setStyleSheet("color: red;");
  ui->serialdevice->setEnabled(true);

  if (communication_error_shown_) {
    return;
  }

  communication_error_shown_ = true;
  QString disconnected = tr("diagnosis unavailable");
  if (dynamixel_ctrl_ != nullptr) {
    const auto ids = dynamixel_ctrl_->find_disconnected_ids();
    if (!ids.empty()) {
      QStringList values;
      for (const uint8_t id : ids) {
        values << QString::number(static_cast<int>(id));
        if (id < dynamixel_labels_.size()) {
          dynamixel_labels_[id]->setStyleSheet(
              "color: white; background-color: rgb(180, 0, 0);");
        }
      }
      disconnected = values.join(", ");
    } else {
      disconnected = tr("none (all IDs responded during individual retry)");
    }
  }
  QMessageBox::critical(
      this, tr("Dynamixel connection lost"),
      tr("Dynamixel communication failed. Motor commands have been stopped.\n"
         "Disconnected ID(s): %1\n"
         "The motion can still be saved.\n\n%2")
          .arg(disconnected, message));
}

bool MainWindow::validate_step_number(int step_number) const {
  return step_number >= 1 &&
         step_number <= static_cast<int>(steps.size());
}

bool MainWindow::validate_step_timing(int time_ms, int delay_ms,
                                      const QString& step_name) {
  if (time_ms >= min_step_time_ms_ && delay_ms >= min_step_delay_ms_) {
    return true;
  }
  const QString prefix = step_name.isEmpty() ? tr("Step") : step_name;
  QMessageBox::warning(
      this, tr("Unsafe step timing"),
      tr("%1 was not accepted.\n"
         "Time must be at least %2 ms and delay must be at least %3 ms.\n"
         "These limits can be changed in config/motor.yaml.")
          .arg(prefix).arg(min_step_time_ms_).arg(min_step_delay_ms_));
  return false;
}

void MainWindow::record_step_history() {
  undo_step_history_.push_back(steps);
  while (undo_step_history_.size() > undo_history_limit_)
    undo_step_history_.pop_front();
  redo_step_history_.clear();
}

void MainWindow::restore_current_step_pointer() {
  current_step = nullptr;
  for (auto& step : steps) {
    if (step.selected) {
      current_step = &step;
      break;
    }
  }
  if (current_step == nullptr && !steps.empty()) {
    steps.back().selected = true;
    current_step = &steps.back();
  }
  ui->step_spinbox->setMaximum(std::max(1, static_cast<int>(steps.size())));
}

void MainWindow::undo_step_edit() {
  if (undo_step_history_.empty()) {
    statusBar()->showMessage(tr("No step edit to undo"), 2000);
    return;
  }
  redo_step_history_.push_back(steps);
  while (redo_step_history_.size() > undo_history_limit_)
    redo_step_history_.pop_front();
  steps = undo_step_history_.back();
  undo_step_history_.pop_back();
  restore_current_step_pointer();
  statusBar()->showMessage(tr("Step edit undone"), 2000);
}

void MainWindow::redo_step_edit() {
  if (redo_step_history_.empty()) {
    statusBar()->showMessage(tr("No step edit to redo"), 2000);
    return;
  }
  undo_step_history_.push_back(steps);
  while (undo_step_history_.size() > undo_history_limit_)
    undo_step_history_.pop_front();
  steps = redo_step_history_.back();
  redo_step_history_.pop_back();
  restore_current_step_pointer();
  statusBar()->showMessage(tr("Step edit redone"), 2000);
}

void MainWindow::select_step_for_edit(int zero_based_index) {
  if (zero_based_index < 0 ||
      zero_based_index >= static_cast<int>(steps.size())) return;
  auto selected = steps.begin();
  std::advance(selected, zero_based_index);
  for (auto& step : steps) step.selected = false;
  selected->selected = true;
  current_step = &(*selected);
  ui->step_spinbox->setValue(zero_based_index + 1);
  ui->step_time->setPlainText(QString::number(selected->time));
  ui->step_delay->setPlainText(QString::number(selected->delay));
  ui->step_repeat->setPlainText(QString::number(selected->repeat));
  ui->step_name->setPlainText(QString::fromStdString(selected->name));
  statusBar()->showMessage(
      tr("Selected step %1 for save/delete").arg(zero_based_index + 1), 2000);
}

bool MainWindow::parse_step_selection(const QString &selection,
                                      std::vector<int> *step_numbers,
                                      QString *error) const {
  step_numbers->clear();
  const QString normalized = selection.trimmed().toLower();
  if (normalized == "all") {
    for (int step = 1; step <= static_cast<int>(steps.size()); ++step) {
      step_numbers->push_back(step);
    }
    return true;
  }

  const QStringList parts = normalized.split(',', Qt::SkipEmptyParts);
  for (const QString &raw_part : parts) {
    const QString part = raw_part.trimmed();
    const QStringList range = part.split('-');
    bool start_ok = false;
    bool end_ok = false;
    const int start = range.value(0).trimmed().toInt(&start_ok);
    const int end = range.size() == 1
                        ? start
                        : range.value(1).trimmed().toInt(&end_ok);
    if (range.size() == 1) end_ok = start_ok;

    if (range.size() > 2 || !start_ok || !end_ok || start > end ||
        !validate_step_number(start) || !validate_step_number(end)) {
      *error = tr("'%1' is invalid. Use step numbers/ranges such as "
                  "1,3,5-8, or enter all.")
                   .arg(part);
      return false;
    }
    for (int step = start; step <= end; ++step) {
      if (std::find(step_numbers->begin(), step_numbers->end(), step) ==
          step_numbers->end()) {
        step_numbers->push_back(step);
      }
    }
  }

  if (step_numbers->empty()) {
    *error = tr("Enter at least one step number, range, or all.");
    return false;
  }
  std::sort(step_numbers->begin(), step_numbers->end());
  return true;
}

void MainWindow::setup_dynamixel_table() {
  QStringList header;
  header << "ID" << "Model" << "Model No." << "Load" << "HW Error"
         << "Temp" << "Voltage";

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
    temp_load_timer_->stop();
    ui->connect->setStyleSheet("color: red;");
    ui->connect->setText("Connect");
    ui->serialdevice->setDisabled(false);
    ui->search_stat->setText("");
    ui->search_process->setValue(0);
    is_connected = false;
    motor_command_active_ = false;
    dynamixel_ids_.clear();
    dynamixel_model_numbers_.clear();
    dynamixel_model_names_.clear();
    dynamixel_types_.clear();
    max_position_limit_.clear();
    min_position_limit_.clear();
    dynamixel_status_timer_->stop();
    delete dynamixel_ctrl_;
    dynamixel_ctrl_ = nullptr;
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
  if (temp_load_timer_ != nullptr && temp_load_timer_->isActive())
    temp_load_timer_->stop();
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
  if (!is_dynamixel_ready()) return;
  const int read_divisor = motor_write_hz_ / motor_read_hz_;
  communication_cycle_count_ =
      (communication_cycle_count_ + 1) % read_divisor;
  const bool read_due = communication_cycle_count_ == 0;
  try {
    const auto *positions = motor_command_active_ ? &pending_goal_positions_ : nullptr;
    const auto *velocities = motor_command_active_ ? &pending_goal_velocities_ : nullptr;
    const auto *accelerations = motor_command_active_ ? &pending_goal_accelerations_ : nullptr;
    dynamixel_ctrl_->communication_cycle(positions, velocities, accelerations,
                                         read_due);
  } catch (const std::exception &e) {
    handle_communication_error(e.what());
    return;
  }
  if (!read_due) return;
  if (ui->dynamixel->rowCount() != static_cast<int>(dynamixel_ids_.size())) {
    ui->dynamixel->clearContents();
    ui->dynamixel->setRowCount(dynamixel_ids_.size());
  }
  for (size_t row = 0; row < dynamixel_ids_.size(); ++row) {
    const auto &motor = dynamixel_ctrl_->dynamixels[row];
    const auto set_item = [this, row](int column, const QString &text) {
      auto *item = ui->dynamixel->item(row, column);
      if (item == nullptr) {
        item = new QTableWidgetItem;
        ui->dynamixel->setItem(row, column, item);
      }
      item->setText(text);
      return item;
    };
    set_item(0, QString::number(motor->get_id()));
    set_item(1, QString::fromStdString(motor->get_model_name()));
    set_item(2, QString::number(motor->get_model_number()));
    const bool current_feedback = motor->has_current_feedback();
    const QString load_unit = current_feedback ? " mA" : " %";
    auto *load = set_item(3, QString::number(motor->get_present_current(), 'f', 1) + load_unit);
    const double ratio = std::min(1.0, motor->get_load_ratio());
    const int red = static_cast<int>(40 + 215 * ratio);
    const int green = static_cast<int>(200 * (1.0 - ratio));
    const QColor load_color(red, green, 30);
    load->setForeground(load_color);
    const int id = motor->get_id();
    if (id >= 0 && id < static_cast<int>(dynamixel_labels_.size())) {
      dynamixel_labels_[id]->setStyleSheet(
          QString("color: white; background-color: rgb(%1, %2, 30);")
              .arg(red).arg(green));
      dynamixel_labels_[id]->setToolTip(
          QString("Load: %1%2 (%3% of limit)")
              .arg(motor->get_present_current(), 0, 'f', 1)
              .arg(load_unit).arg(ratio * 100.0, 0, 'f', 0));
    }
    auto *error = set_item(4, QString("0x%1").arg(motor->get_error_status(), 2, 16, QChar('0')));
    error->setForeground(motor->get_error_status() == 0 ? Qt::green : Qt::red);
    set_item(5, QString::number(motor->get_present_temperature(), 'f', 0) + " C");
    set_item(6, QString::number(motor->get_present_voltage(), 'f', 1) + " V");
  }
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

void MainWindow::on_set_all_zero_clicked() {
  if (!is_dynamixel_ready()) {
    QMessageBox::warning(this, tr("Dynamixel not connected"),
                         tr("Connect Dynamixel before moving all motors."));
    return;
  }
  if (QMessageBox::question(
          this, tr("Set all motors to zero"),
          tr("Move every connected motor to 0 degrees over 1000 ms?"),
          QMessageBox::Yes | QMessageBox::No,
          QMessageBox::No) != QMessageBox::Yes) {
    return;
  }

  temp_load_start_positions_ = dynamixel_goal_positions_;
  temp_load_target_positions_.assign(dynamixel_goal_positions_.size(), 0.0);
  temp_load_elapsed_ms_ = 0;
  temp_load_timer_->start();
  for (const auto& motor : dynamixel_ctrl_->dynamixels) {
    const int id = motor->get_id();
    if (id >= 0 && id < static_cast<int>(dynamixel_buttons_.size()))
      dynamixel_buttons_[id]->setText("0");
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
  if (!validate_step_timing(step.time, step.delay, tr("New step"))) return;

  for (int i = 0; i < dynamixel_ctrl_->dynamixels.size(); i++) {
    Dynamixel_postition dxl;
    dxl.id = dynamixel_ctrl_->dynamixels[i]->get_id();
    dxl.position = dynamixel_goal_positions_[i];
    step.dxl.push_back(dxl);
  }

  auto insert_position = steps.end();
  if (current_step != nullptr) {
    const auto selected = std::find_if(
        steps.begin(), steps.end(),
        [this](Step& item) { return &item == current_step; });
    if (selected != steps.end()) insert_position = std::next(selected);
  }

  record_step_history();
  for (std::list<Step>::iterator it = steps.begin(); it != steps.end(); it++) {
    it->selected = false;
  }

  const auto inserted = steps.insert(insert_position, step);
  current_step = &(*inserted);
  const int inserted_number =
      static_cast<int>(std::distance(steps.begin(), inserted)) + 1;
  ui->step_spinbox->setMaximum(static_cast<int>(steps.size()));
  ui->step_spinbox->setValue(inserted_number);
  statusBar()->showMessage(
      tr("Saved new step at position %1").arg(inserted_number), 2000);
}

void MainWindow::on_delete_step_clicked() {
  if (steps.size() == 0) {
    std::cout << "No step to delete" << std::endl;
    return;
  }

  auto target = steps.end();
  if (current_step != nullptr) {
    target = std::find_if(steps.begin(), steps.end(), [this](Step& step) {
      return &step == current_step;
    });
  }
  if (target == steps.end()) {
    const int number = ui->step_spinbox->value();
    if (!validate_step_number(number)) return;
    target = steps.begin();
    std::advance(target, number - 1);
  }
  const int target_number =
      static_cast<int>(std::distance(steps.begin(), target)) + 1;
  if (QMessageBox::question(
          this, tr("Delete selected step"),
          tr("Delete step %1 (%2)?\nThis can be undone with Ctrl+Z.")
              .arg(target_number)
              .arg(QString::fromStdString(target->name))) != QMessageBox::Yes) {
    return;
  }
  record_step_history();
  auto next = steps.erase(target);
  current_step = nullptr;

  if (steps.size() == 0) {
    return;
  }
  if (next == steps.end()) --next;
  for (auto& step : steps) step.selected = false;
  next->selected = true;
  current_step = &(*next);
  ui->step_spinbox->setValue(
      static_cast<int>(std::distance(steps.begin(), next)) + 1);
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
  if (!validate_step_timing(step.time, step.delay,
                            tr("Step %1").arg(step_number))) return;

  for (int i = 0; i < dynamixel_ctrl_->dynamixels.size(); i++) {
    Dynamixel_postition dxl;
    dxl.id = dynamixel_ctrl_->dynamixels[i]->get_id();
    dxl.position = dynamixel_goal_positions_[i];
    step.dxl.push_back(dxl);
  }

  record_step_history();
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

void MainWindow::on_apply_step_delta_clicked() {
  const int source_step_number = ui->step_spinbox->value();
  if (!validate_step_number(source_step_number)) {
    QMessageBox::warning(this, tr("Invalid source step"),
                         tr("Select a source step between 1 and %1 first.")
                             .arg(static_cast<int>(steps.size())));
    return;
  }
  if (!is_dynamixel_ready()) {
    QMessageBox::warning(this, tr("Dynamixel not connected"),
                         tr("Connect Dynamixel before applying motor changes."));
    return;
  }

  auto source = steps.begin();
  std::advance(source, source_step_number - 1);
  struct MotorDelta {
    uint8_t id;
    double value;
  };
  std::vector<MotorDelta> deltas;
  for (size_t motor_index = 0;
       motor_index < dynamixel_ctrl_->dynamixels.size(); ++motor_index) {
    const uint8_t id = dynamixel_ctrl_->dynamixels[motor_index]->get_id();
    const auto saved = std::find_if(
        source->dxl.begin(), source->dxl.end(),
        [id](const Dynamixel_postition &position) { return position.id == id; });
    if (saved == source->dxl.end()) continue;

    const double delta = dynamixel_goal_positions_[motor_index] - saved->position;
    if (std::fabs(delta) > 1e-9) deltas.push_back({id, delta});
  }

  if (deltas.empty()) {
    QMessageBox::information(
        this, tr("No motor changes"),
        tr("The current motor positions do not differ from source step %1.")
            .arg(source_step_number));
    return;
  }

  bool accepted = false;
  const QString selection = QInputDialog::getText(
      this, tr("Apply motor changes"),
      tr("Target steps (example: 1,3,5-8 or all):"), QLineEdit::Normal,
      "all", &accepted);
  if (!accepted) return;

  std::vector<int> target_steps;
  QString parse_error;
  if (!parse_step_selection(selection, &target_steps, &parse_error)) {
    QMessageBox::warning(this, tr("Invalid target steps"), parse_error);
    return;
  }

  // Validate every resulting position first so the operation is atomic.
  for (int step_number : target_steps) {
    auto target = steps.begin();
    std::advance(target, step_number - 1);
    for (const MotorDelta &delta : deltas) {
      const auto position = std::find_if(
          target->dxl.begin(), target->dxl.end(),
          [&delta](const Dynamixel_postition &item) {
            return item.id == delta.id;
          });
      if (position == target->dxl.end()) continue;

      const auto motor = std::find_if(
          dynamixel_ctrl_->dynamixels.begin(),
          dynamixel_ctrl_->dynamixels.end(),
          [&delta](const auto &item) { return item->get_id() == delta.id; });
      if (motor == dynamixel_ctrl_->dynamixels.end()) continue;
      const size_t motor_index =
          std::distance(dynamixel_ctrl_->dynamixels.begin(), motor);
      const double result = position->position + delta.value;
      if (result < min_position_limit_[motor_index] ||
          result > max_position_limit_[motor_index]) {
        QMessageBox::warning(
            this, tr("Motor limit exceeded"),
            tr("Step %1, motor %2 would become %3 degrees. No steps were changed.")
                .arg(step_number)
                .arg(delta.id)
                .arg(RAD2DEG(result), 0, 'f', 2));
        return;
      }
    }
  }

  record_step_history();
  for (int step_number : target_steps) {
    auto target = steps.begin();
    std::advance(target, step_number - 1);
    for (const MotorDelta &delta : deltas) {
      const auto position = std::find_if(
          target->dxl.begin(), target->dxl.end(),
          [&delta](const Dynamixel_postition &item) {
            return item.id == delta.id;
          });
      if (position != target->dxl.end()) position->position += delta.value;
    }
  }

  QMessageBox::information(
      this, tr("Motor changes applied"),
      tr("Applied changes for %1 motor(s) to %2 step(s).")
          .arg(static_cast<int>(deltas.size()))
          .arg(static_cast<int>(target_steps.size())));
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

  temp_load_start_positions_ = dynamixel_goal_positions_;
  temp_load_target_positions_ = dynamixel_goal_positions_buff_;
  temp_load_elapsed_ms_ = 0;
  temp_load_timer_->start();

  for (int i = 0; i < dynamixel_types_.size(); i++) {
    int _id = static_cast<int>(dynamixel_ctrl_->dynamixels[i]->get_id());
    std::string value_ =
        std::to_string(static_cast<int>(
            RAD2DEG(temp_load_target_positions_[i])));
    dynamixel_buttons_[_id]->setText(value_.c_str());
  }
}

void MainWindow::oper_dynamixel() {
  if (!is_dynamixel_ready()) {
    return;
  }

  pending_goal_positions_ = dynamixel_goal_positions_;
  pending_goal_velocities_ = dynamixel_goal_velocities_;
  pending_goal_accelerations_ = dynamixel_goal_accelerations_;
  motor_command_active_ = true;
}

void MainWindow::oper_dynamixel_V(std::list<Step>::iterator it) {
  if (!is_dynamixel_ready() || it == steps.end()) {
    return;
  }

  if (!validate_step_timing(it->time, it->delay,
                            QString::fromStdString(it->name))) return;

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

  if (!validate_step_timing(init_step->time, init_step->delay,
                            tr("Init step"))) return;

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

  record_step_history();
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

  const int time_ms = ui->step_time->toPlainText().toInt();
  const int delay_ms = ui->step_delay->toPlainText().toInt();
  if (!validate_step_timing(time_ms, delay_ms, tr("Init step"))) return;

  init_step.reset(new Step);
  init_step->time = time_ms;
  init_step->delay = delay_ms;
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
  // Keep the Qt event loop alive while a step is in progress.  The motor
  // communication loop is a precise QTimer, so blocking this thread with
  // sleep_for prevents every queued write/read from running.
  const auto wait_with_communication = [](int duration_ms) {
    if (duration_ms <= 0) return;
    QEventLoop loop;
    QTimer timer;
    timer.setSingleShot(true);
    QObject::connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);
    timer.start(duration_ms);
    loop.exec();
  };

  ui->motion_operate->setEnabled(false);
  std::list<Step>::iterator it = steps.begin();

  for (int i = 0; i < steps.size(); i++) {
    oper_dynamixel_V(it);
    if (!is_dynamixel_ready()) {
      ui->motion_operate->setEnabled(true);
      return;
    }
    std::cout << "time : " << it->time << " delay : " << it->delay
              << " repeat : " << it->repeat << std::endl;

    std::cout << "go time" << std::endl;
    wait_with_communication(it->time);
    if (!is_dynamixel_ready()) {
      ui->motion_operate->setEnabled(true);
      return;
    }
    std::cout << "end time" << std::endl;
    std::cout << "go delay" << std::endl;
    wait_with_communication(it->delay);
    std::cout << "end delay" << std::endl;
    it++;
  }
  ui->motion_operate->setEnabled(true);
}

QString MainWindow::next_copy_path(const QString& original) const {
  const QFileInfo info(original);
  const QString directory = info.absolutePath();
  const QString suffix = info.suffix().isEmpty() ? QString() : "." + info.suffix();
  const QString base = info.completeBaseName();
  for (int copy = 1;; ++copy) {
    const QString candidate =
        directory + "/" + base + QString("(%1)").arg(copy) + suffix;
    if (!QFileInfo::exists(candidate)) return candidate;
  }
}

Step MainWindow::mirror_step(const Step& source, const Step& reference) const {
  Step mirrored = source;
  mirrored.selected = false;

  const auto find_position = [](const Step& step, uint8_t id)
      -> const Dynamixel_postition* {
    const auto found = std::find_if(
        step.dxl.begin(), step.dxl.end(),
        [id](const Dynamixel_postition& item) { return item.id == id; });
    return found == step.dxl.end() ? nullptr : &(*found);
  };
  const auto find_mutable_position = [](Step& step, uint8_t id)
      -> Dynamixel_postition* {
    const auto found = std::find_if(
        step.dxl.begin(), step.dxl.end(),
        [id](const Dynamixel_postition& item) { return item.id == id; });
    return found == step.dxl.end() ? nullptr : &(*found);
  };

  for (const MirrorPair& pair : mirror_pairs_) {
    const auto *source_left = find_position(source, pair.left_id);
    const auto *source_right = find_position(source, pair.right_id);
    const auto *init_left = find_position(reference, pair.left_id);
    const auto *init_right = find_position(reference, pair.right_id);
    auto *result_left = find_mutable_position(mirrored, pair.left_id);
    auto *result_right = find_mutable_position(mirrored, pair.right_id);
    if (source_left == nullptr || source_right == nullptr ||
        init_left == nullptr || init_right == nullptr ||
        result_left == nullptr || result_right == nullptr) {
      continue;
    }

    // YAML stores absolute radians.  Exchange only the displacement from the
    // reference pose, preserving asymmetric INIT offsets and motor mounting.
    result_left->position = init_left->position + pair.direction *
        (source_right->position - init_right->position);
    result_right->position = init_right->position + pair.direction *
        (source_left->position - init_left->position);
  }
  return mirrored;
}

void MainWindow::open_mirror_editor() {
  if (steps.empty()) {
    QMessageBox::information(this, tr("Left/Right Mirror"),
                             tr("Load or create motion steps first."));
    return;
  }
  if (mirror_pairs_.empty()) {
    QMessageBox::warning(
        this, tr("Left/Right Mirror"),
        tr("No mirror_pairs are configured in config/motor.yaml."));
    return;
  }

  QDialog dialog(this);
  dialog.setWindowTitle(tr("Left/Right Motion Mirror"));
  auto *layout = new QFormLayout(&dialog);
  auto *reference = new QComboBox(&dialog);
  if (init_step != nullptr && !init_step->dxl.empty()) {
    reference->addItem(tr("Saved INIT pose"), -1);
  }
  int number = 1;
  for (const Step& step : steps) {
    reference->addItem(
        tr("Step %1 — %2").arg(number).arg(QString::fromStdString(step.name)),
        number);
    ++number;
  }
  auto *targets = new QLineEdit("all", &dialog);
  targets->setPlaceholderText(tr("Example: 1,3,5-8 or all"));
  auto *mode = new QComboBox(&dialog);
  mode->addItem(tr("Append mirrored steps"), "append");
  mode->addItem(tr("Replace selected steps"), "replace");
  layout->addRow(tr("INIT reference pose"), reference);
  layout->addRow(tr("Steps to mirror"), targets);
  layout->addRow(tr("Result"), mode);
  auto *description = new QLabel(
      tr("Time, delay and repeat are preserved. IDs 22 and 23 remain unchanged.\n"
         "The operation can be undone with Undo/Ctrl+Z."), &dialog);
  description->setWordWrap(true);
  layout->addRow(description);
  auto *buttons = new QDialogButtonBox(
      QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  layout->addRow(buttons);
  if (dialog.exec() != QDialog::Accepted) return;

  std::vector<int> selected_steps;
  QString parse_error;
  if (!parse_step_selection(targets->text(), &selected_steps, &parse_error)) {
    QMessageBox::warning(this, tr("Invalid target steps"), parse_error);
    return;
  }

  const int reference_number = reference->currentData().toInt();
  Step reference_pose;
  if (reference_number == -1) {
    reference_pose = *init_step;
  } else {
    auto it = steps.begin();
    std::advance(it, reference_number - 1);
    reference_pose = *it;
  }

  std::vector<Step> results;
  results.reserve(selected_steps.size());
  for (const int step_number : selected_steps) {
    auto source = steps.begin();
    std::advance(source, step_number - 1);
    Step mirrored = mirror_step(*source, reference_pose);
    mirrored.name += " (mirrored)";
    results.push_back(std::move(mirrored));
  }

  record_step_history();
  if (mode->currentData().toString() == "append") {
    for (Step& result : results) steps.push_back(std::move(result));
  } else {
    for (std::size_t i = 0; i < selected_steps.size(); ++i) {
      auto target = steps.begin();
      std::advance(target, selected_steps[i] - 1);
      *target = std::move(results[i]);
    }
  }
  restore_current_step_pointer();
  ui->step_spinbox->setMaximum(static_cast<int>(steps.size()));
  statusBar()->showMessage(
      tr("Created %1 mirrored step(s)").arg(selected_steps.size()), 3000);
}

void MainWindow::add_recent_motion(const QString& path) {
  const QString absolute = QFileInfo(path).absoluteFilePath();
  recent_motion_files_.removeAll(absolute);
  recent_motion_files_.prepend(absolute);
  while (recent_motion_files_.size() > 10) recent_motion_files_.removeLast();
  QSettings settings("ROBIT", "HumanoidMotionTeacher");
  settings.setValue("recentMotionFiles", recent_motion_files_);
  rebuild_recent_motion_menu();
}

void MainWindow::rebuild_recent_motion_menu() {
  if (recent_motion_menu_ == nullptr) return;
  recent_motion_menu_->clear();
  for (const QString& path : recent_motion_files_) {
    auto *action = recent_motion_menu_->addAction(QFileInfo(path).fileName());
    action->setToolTip(path);
    connect(action, &QAction::triggered, this,
            [this, path]() { open_motion_file(path); });
  }
  if (recent_motion_files_.empty()) {
    auto *empty = recent_motion_menu_->addAction(tr("No recent motions"));
    empty->setEnabled(false);
  }
}

bool MainWindow::open_motion_file(const QString& path) {
  try {
    loadMotionFromYaml(path.toStdString());
    file_path = path.toStdString();
    ui->file_path->setText(path);
    add_recent_motion(path);
    return true;
  } catch (const std::exception &e) {
    recent_motion_files_.removeAll(QFileInfo(path).absoluteFilePath());
    QSettings settings("ROBIT", "HumanoidMotionTeacher");
    settings.setValue("recentMotionFiles", recent_motion_files_);
    rebuild_recent_motion_menu();
    QMessageBox::critical(this, tr("Load failed"), QString::fromUtf8(e.what()));
    return false;
  }
}

void MainWindow::saveMotionFile() {
  const QString current = QString::fromStdString(file_path);
  const QFileInfo info(current);
  if (current.isEmpty() || !info.exists() || !info.isFile()) {
    saveMotionFileas();
    return;
  }

  QMessageBox prompt(QMessageBox::Question, tr("Save motion"),
                     tr("Save a copy and keep the original file unchanged?"),
                     QMessageBox::NoButton, this);
  auto *copy_button = prompt.addButton(tr("Save Copy"), QMessageBox::YesRole);
  auto *overwrite_button = prompt.addButton(tr("Overwrite Original"), QMessageBox::NoRole);
  prompt.addButton(QMessageBox::Cancel);
  prompt.exec();
  if (prompt.clickedButton() == copy_button) {
    const QString copy_path = next_copy_path(current);
    saveMotionToYaml(copy_path.toStdString(), steps, true);
    add_recent_motion(copy_path);
    QMessageBox::information(this, tr("Copy saved"), copy_path);
  } else if (prompt.clickedButton() == overwrite_button) {
    saveMotionToYaml(file_path, steps, true);
    add_recent_motion(current);
    statusBar()->showMessage(tr("Motion saved: %1").arg(current), 3000);
  }
}

void MainWindow::saveMotionFileas() {  // 파일 저장 대화상자 표시
  QString fileName = QFileDialog::getSaveFileName(
      this, tr("Save Motion File"), QString::fromStdString(file_path),
      tr("YAML Files (*.yaml);;All Files (*)"));
  if (fileName.isEmpty()) {
    return;
  }

  saveMotionToYaml(fileName.toStdString(), steps, true);

  file_path = fileName.toStdString();
  ui->file_path->setText(fileName);
  add_recent_motion(fileName);
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
    if (sender() == ui->actionOpen_Step_file) {
      try {
        loadMotionFromYaml(fileName.toStdString());
      } catch (const std::exception &e) {
        QMessageBox::critical(this, tr("Load failed"),
                              QString::fromUtf8(e.what()));
      }
    } else {
      open_motion_file(fileName);
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
      if (step.time < min_step_time_ms_ || step.delay < min_step_delay_ms_) {
        throw std::runtime_error(
            "Motion file step " + std::to_string(i - 2) +
            " is below the configured minimum time/delay");
      }
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
      if (step.time < min_step_time_ms_ || step.delay < min_step_delay_ms_) {
        throw std::runtime_error(
            "Motion file step " + std::to_string(i - 2) +
            " is below the configured minimum time/delay");
      }
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
