// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <string>
#include "communication/identifiable.h"
#include "utils/pose.h"

using std::string;

using namespace depth_clustering;

class TestClass : public Identifiable {
 public:
  TestClass() : Identifiable() {}
  virtual ~TestClass() {}

 protected:
};

template <class T>
class TestTemplateClass : public Identifiable {
 public:
  TestTemplateClass() : Identifiable() {}
  virtual ~TestTemplateClass() {}

 protected:
};

TEST(IdentifiableTest, Init) {
  int init_counter = TestClass::get_current_id_counter();
  TestClass obj1;
  TestClass obj2;
  TestTemplateClass<int> obj3;
  TestTemplateClass<double> obj4;
  TestTemplateClass<TestClass> obj5;
  string name = "TestClass";
  string name_int = "TestTemplateClass<int>";
  string name_double = "TestTemplateClass<double>";
  string name_test_class = "TestTemplateClass<TestClass>";
  ASSERT_EQ(init_counter + 1, obj1.id());
  ASSERT_EQ(init_counter + 2, obj2.id());
  ASSERT_EQ(init_counter + 3, obj3.id());
  ASSERT_EQ(init_counter + 4, obj4.id());
  ASSERT_EQ(init_counter + 5, obj5.id());
  ASSERT_EQ(name, obj1.guess_class_name());
  ASSERT_EQ(name, obj2.guess_class_name());
  ASSERT_EQ(name_int, obj3.guess_class_name());
  ASSERT_EQ(name_double, obj4.guess_class_name());
  ASSERT_EQ(name_test_class, obj5.guess_class_name());
}
