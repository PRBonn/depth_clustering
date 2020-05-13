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
#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "communication/identifiable.h"
#include "utils/pose.h"

using std::string;

using namespace depth_clustering;

class TestSender : public AbstractSender<int> {
 public:
  TestSender() : AbstractSender<int>(SenderType::STORER) {}
  virtual ~TestSender() {}

  // override a protected member for testing purposes
  void ShareDataWithAllClients(const int& obj) {
    AbstractSender<int>::ShareDataWithAllClients(obj);
  }
};

class TestClient : public AbstractClient<int> {
 public:
  TestClient()
      : AbstractClient<int>(), _object_received(false), _sender_id(-1) {}
  virtual ~TestClient() {}

  void OnNewObjectReceived(const int&, const int sender_id) override {
    _object_received = true;
    _sender_id = sender_id;
  }

  bool _object_received;
  int _sender_id;
};

template <class T>
class TestTemplateClient : public AbstractClient<T> {
 public:
  TestTemplateClient() : AbstractClient<T>(), _object_received(false) {}
  virtual ~TestTemplateClient() {}

  void OnNewObjectReceived(const T&, const int) override {
    _object_received = true;
  }

  bool _object_received;
};

TEST(SenderClientTest, Init) {
  int id = Identifiable::get_current_id_counter();
  TestSender sender;
  TestClient client_simple_1;
  TestClient client_simple_2;
  TestTemplateClient<int> client_templated;
  string name_sender = "TestSender";
  string name_client_simple_1 = "TestClient";
  string name_client_simple_2 = "TestClient";
  string name_client_templated = "TestTemplateClient<int>";
  ASSERT_EQ(id + 1, sender.id());
  ASSERT_EQ(id + 2, client_simple_1.id());
  ASSERT_EQ(id + 3, client_simple_2.id());
  ASSERT_EQ(id + 4, client_templated.id());
  ASSERT_EQ(name_sender, sender.guess_class_name());
  ASSERT_EQ(name_client_simple_1, client_simple_1.guess_class_name());
  ASSERT_EQ(name_client_simple_2, client_simple_2.guess_class_name());
  ASSERT_EQ(name_client_templated, client_templated.guess_class_name());
}

TEST(SenderClientTest, SendAndReceive) {
  int id = Identifiable::get_current_id_counter();
  TestSender sender;
  TestClient client_simple_1;
  TestClient client_simple_2;
  TestTemplateClient<int> client_templated;
  ASSERT_EQ(id + 1, sender.id());
  ASSERT_EQ(id + 2, client_simple_1.id());
  ASSERT_EQ(id + 3, client_simple_2.id());
  ASSERT_EQ(id + 4, client_templated.id());

  ASSERT_EQ(0ul, sender.client_count());
  sender.AddClient(&client_simple_1);
  ASSERT_EQ(1ul, sender.client_count());
  sender.AddClient(&client_simple_2);
  ASSERT_EQ(2ul, sender.client_count());
  sender.AddClient(&client_templated);
  ASSERT_EQ(3ul, sender.client_count());

  sender.RemoveClient(client_simple_2.id());
  ASSERT_EQ(2ul, sender.client_count());
  int package = 42;
  ASSERT_EQ(false, client_simple_1._object_received);
  ASSERT_EQ(false, client_simple_2._object_received);
  ASSERT_EQ(false, client_templated._object_received);
  ASSERT_EQ(-1, client_simple_1._sender_id);

  sender.ShareDataWithAllClients(package);
  ASSERT_EQ(true, client_simple_1._object_received);
  ASSERT_EQ(false, client_simple_2._object_received);
  ASSERT_EQ(true, client_templated._object_received);

  ASSERT_EQ(sender.id(), client_simple_1._sender_id);
}

TEST(SenderClientTest, RemoveNonExistingId) {
  TestSender sender;
  ASSERT_EQ(0ul, sender.client_count());
  sender.RemoveClient(42);
  ASSERT_EQ(0ul, sender.client_count());
}
