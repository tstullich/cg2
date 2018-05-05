#ifndef SIDEBARWIDGET_HPP
#define SIDEBARWIDGET_HPP

#include <QComboBox>
#include <QDockWidget>
#include <QSlider>
#include <QString>
#include <QVBoxLayout>

class SidebarWidget : public QDockWidget {
  Q_OBJECT
  public:
  SidebarWidget(QWidget *parent = 0);

  private:
  QComboBox *dropdownMenu;
  QSlider *slider;
  QVBoxLayout *layout;
  QWidget *container;
};
#endif // SIDEBARWIDGET_HPP
