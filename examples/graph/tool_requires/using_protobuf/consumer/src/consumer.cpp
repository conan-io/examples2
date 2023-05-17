#include <iostream>
#include "consumer.h"
#include "addressbook.pb.h"


void consumer(){
    tutorial::AddressBook address_book;
    auto * person = address_book.add_people();
    person->set_id(1337);
    std::cout << "Consumer(): created a person with id 1337\n";
}
