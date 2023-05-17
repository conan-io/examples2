#include <iostream>
#include <fstream>
#include <string>
#include "addressbook.pb.h"


int main(int argc, char* argv[]) {

  GOOGLE_PROTOBUF_VERIFY_VERSION;


  tutorial::AddressBook address_book;
  auto * person = address_book.add_people();
  person->set_id(1337);

  // Optional:  Delete all global objects allocated by libprotobuf.
  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}