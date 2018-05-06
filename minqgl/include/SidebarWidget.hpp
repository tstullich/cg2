#ifndef SIDEBARWIDGET_HPP
#define SIDEBARWIDGET_HPP

#include <QComboBox>
#include <QDockWidget>
#include <QLabel>
#include <QSlider>
#include <QString>
#include <QVBoxLayout>

class SidebarWidget : public QDockWidget {
  Q_OBJECT
  public:
  SidebarWidget(QWidget *parent = 0);

  /*
   * Retrieves the current mode of displaying
   * the K-d tree.
   * The options are as follows:
   * 0 - Hyper plane mode
   * 1 - Collect in radius mode
   * 2 - K-Nearest neighbor mode
   */
  int getCurrentMode() const;

  // Sets the callback from our QGLViewerWidget
  // So when a value changes in the slider we
  // can adjust K-d tree drawing accordingly
  // We might not need this in the future
  // but I am keeping it for now
  void setSliderCallback(QWidget *w);

  private slots:
  /*
   * A slot that will help update our slider ranges
   * The option value should map to the following:
   * 0 - Hyper plane mode
   * 1 - Collect in radius mode
   * 2 - K-Nearest neighbor mode
   */
  void updateSliderValues(int option);

  private:
  void setSliderRange(int minVal, int maxVal);

  const int HYPER_PLANE_DEFAULT_MIN = 0;
  const int HYPER_PLANE_DEFAULT_MAX = 8;

  const int COLLECT_IN_RADIUS_MIN = 1;
  const int COLLECT_IN_RADIUS_MAX = 10;

  const int K_NEAREST_MIN = 1;
  const int K_NEAREST_MAX = 20;

  QComboBox *dropdownMenu;
  QLabel *startLabel;
  QLabel *endLabel;
  QSlider *slider;
  QVBoxLayout *layout;
  QWidget *container;
};
#endif // SIDEBARWIDGET_HPP
