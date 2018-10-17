////////////////////////////////
/// usage : 1.	data that identifies a guillotine cut problem and its solution.
/// 
/// note  : 1.	
////////////////////////////////

#ifndef SMART_QYM_INVENTORY_ROUTING_PROBLEM_H
#define SMART_QYM_INVENTORY_ROUTING_PROBLEM_H


#include "Config.h"

#include <iostream>
#include <fstream>
#include <sstream>

#include "Common.h"
#include "PbReader.h"
#include "InventoryRouting.pb.h"


namespace szx {

class Problem {
    #pragma region Type
public:
    using Price = double;

    struct Input : public pb::InventoryRouting::Input {
        bool load(const String &path) { return pb::load(path, *this); }
    };

    struct Output : public pb::InventoryRouting::Output {
        bool save(const String &path, pb::Submission &submission) const {
            std::ofstream ofs(path);
            if (!ofs.is_open()) { return false; }

            // TODO[0]: fill the submission information.
            submission.set_author("qym");
            submission.set_algorithm("rand");
            submission.set_cpu("Intel Core i5-7400 3.00GHz");
            submission.set_ram("16G 2400MHz");
            submission.set_language("C++");
            submission.set_compiler("VS2017");
            submission.set_os("Windows 10");
            submission.set_problem("InventoryRouting");

            ofs << protobufToJson(submission, false) << std::endl << protobufToJson(*this);
            return true;
        }

        Price totalCost = 0;
    };
    #pragma endregion Type

    #pragma region Constant
public:
    #pragma endregion Constant

    #pragma region Constructor
public:
    #pragma endregion Constructor

    #pragma region Method
public:
    #pragma endregion Method

    #pragma region Field
public:
    #pragma endregion Field
}; // Problem

}


#endif // SMART_QYM_INVENTORY_ROUTING_PROBLEM_H
