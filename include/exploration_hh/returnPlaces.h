#include <string>
#include <vector>

#include <database_interface/db_class.h>

class returnPlaces : public database_interface::DBClass
{
public:
  database_interface::DBField<int> id_;
  database_interface::DBField<double> pos_x_;
  database_interface::DBField<double> pos_y_;
  database_interface::DBField<double> prob_;

  returnPlaces () :
    id_(database_interface::DBFieldBase::TEXT,this,"key_column","places2",true),
    pos_x_(database_interface::DBFieldBase::TEXT,this,"pos_x","places2",true),
    pos_y_(database_interface::DBFieldBase::TEXT,this,"pos_y","places2",true),
    prob_(database_interface::DBFieldBase::TEXT,this,"probability","places2",true)
  {
    primary_key_field_ = &id_;
    fields_.push_back(&pos_x_);
    fields_.push_back(&pos_y_);
    fields_.push_back(&prob_);
  }
};


