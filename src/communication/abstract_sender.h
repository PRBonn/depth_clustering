// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SRC_COMMUNICATION_ABSTRACT_SENDER_H_
#define SRC_COMMUNICATION_ABSTRACT_SENDER_H_

#include <map>
#include <string>
#include <typeinfo>
#include <memory>

#include "communication/identifiable.h"
#include "communication/abstract_client.h"

namespace depth_clustering {

enum class SenderType {
  STORER,    // has a copy of the data (at least for some time)
  STREAMER,  // streams data and therefore blocks execution
  UNDEFINED  // if at least one class is undefined -> exit with error
};

/**
 * @brief      Class for abstract sender.
 *
 * @tparam     ObjSendType  Type of the object to be sent.
 */
template <class ObjSendType>
class AbstractSender : public virtual Identifiable {
 public:
  explicit AbstractSender(SenderType type = SenderType::UNDEFINED)
      : Identifiable(), _type(type) {}
  virtual ~AbstractSender() {}

  /**
   * @brief      Gets type of sender as string.
   *
   * @return     Type as string
   */
  const char* type() const {
    // every child should implement this to tell us if it is going to block
    // execution of others when receiving data
    switch (_type) {
      case SenderType::STORER:
        return "STORER";
      case SenderType::STREAMER:
        return "STREAMER";
      default:
        return "undefined";
    }
  }

  /**
   * @brief      Adds a client.
   *
   * @param      client  The client to receive the processed data
   */
  void AddClient(AbstractClient<ObjSendType>* client) {
    if (_type == SenderType::UNDEFINED) {
      fprintf(stderr, "ERROR: class %s (id: %d) has UNDEFINED type;\n",
              this->guess_class_name().c_str(), this->id());
      fprintf(stderr, "\tSet type in AbstractClient constructor;\n");
      fprintf(stderr, "\tType must be one of enum %s:\n",
              DemangleName(typeid(SenderType).name()).c_str());
      fprintf(stderr, "\tExiting...\n");
      exit(EXIT_FAILURE);
    }
    _clients[client->id()] = client;
    fprintf(stderr,
            "\n"
            " ===================== Setting Connection =====================\n"
            "|| Sender: %s (id: %d)\n"
            "|| Type:   %s\n"
            "|| \t\t\t\t|\n"
            "|| \t\t\t\tV\n"
            "|| Client: %s (id: %d)\n"
            " ==============================================================\n",
            this->guess_class_name().c_str(), this->id(), this->type(),
            client->guess_class_name().c_str(), client->id());
  }

  /**
   * @brief      Removes a client.
   *
   * @param[in]  id    The identifier of the client to remove
   */
  void RemoveClient(int id) {
    if (_clients.find(id) == _clients.end()) {
      fprintf(stderr, "Error: no such client to delete: %d\n", id);
      return;
    }
    auto* client = _clients[id];
    fprintf(
        stderr,
        "\n"
        " xxxxxxxxxxxxxxxxxxxxxx Removing Connection xxxxxxxxxxxxxxxxxxxx\n"
        "|| Sender: %s (id: %d)\n"
        "|| \t\t\\/\n"
        "|| \t\t/\\\n"
        "|| Client: %s (id: %d)\n"
        " xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n",
        this->guess_class_name().c_str(), this->id(),
        client->guess_class_name().c_str(), client->id());
    _clients.erase(id);
  }

  void RemoveClient(AbstractClient<ObjSendType>* client) {
    this->RemoveClient(client->id());
  }

  /**
   * @brief      Get number of clients currently subscribed
   *
   * @return     Current number of Clients
   */
  size_t client_count() { return _clients.size(); }

  /**
   * @brief      Shares data with everyone who listens to it.
   *
   * @param[in]  obj   Object to share
   * @param[in]  id    Id, by default this->id()
   */
  void ShareDataWithAllClients(const ObjSendType& obj, int id = -1) {
    int id_to_use = id >= 0 ? id : this->id();
    for (auto& kv : _clients) {
      kv.second->OnNewObjectReceived(obj, id_to_use);
    }
  }

 protected:
  using IdClientMapping = std::map<int, AbstractClient<ObjSendType>*>;

  IdClientMapping _clients;
  SenderType _type;
};

}  // namespace depth_clustering

#endif  // SRC_COMMUNICATION_ABSTRACT_SENDER_H_
