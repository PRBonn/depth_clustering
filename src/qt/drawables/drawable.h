// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#ifndef SRC_QT_DRAWABLES_DRAWABLE_H_
#define SRC_QT_DRAWABLES_DRAWABLE_H_

#include <memory>

class Drawable {
 public:
  using Ptr = std::shared_ptr<Drawable>;
  virtual void Draw() const = 0;
  virtual ~Drawable() {}
};
#endif  // SRC_QT_DRAWABLES_DRAWABLE_H_
