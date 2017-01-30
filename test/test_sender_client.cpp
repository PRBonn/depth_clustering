// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

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

  void OnNewObjectReceived(const int& object, const int sender_id) override {
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

  void OnNewObjectReceived(const T& object, const int sender_id) override {
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

  ASSERT_EQ(0, sender.client_count());
  sender.AddClient(&client_simple_1);
  ASSERT_EQ(1, sender.client_count());
  sender.AddClient(&client_simple_2);
  ASSERT_EQ(2, sender.client_count());
  sender.AddClient(&client_templated);
  ASSERT_EQ(3, sender.client_count());

  sender.RemoveClient(client_simple_2.id());
  ASSERT_EQ(2, sender.client_count());
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
  ASSERT_EQ(0, sender.client_count());
  sender.RemoveClient(42);
  ASSERT_EQ(0, sender.client_count());
}
