// Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

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
