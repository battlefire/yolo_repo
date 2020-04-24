#ifndef json_obj_box
#include "uartwireless.hpp"
#include <json.h>
#include <vector>
#include <iostream>
#include <string>

#define HEAD_LENTH_4G 3
#define INDEX_NAME_LENTH 32

typedef struct
{
	int frame_num;
	int modul_id;
	int type;
	float x_loca;
	float y_loca;
	float high;
	float wide;
	float speed;
	float direction;
	float longitude;
	float latitude;
} road_data_to_server;

class json_obj_box
{
public:
	json_obj_box()
	{
	}
	void datapack_vehicle_json_packge_and_updata();
	void updata_4G();
	void set_data_vehicle_and_datavector(int vehicle_num, vector<road_data_to_server> &Vdata);
	void empty_data_vector();
	string json_send_str;

private:
	int data_vehicle_num;
	vector<road_data_to_server> data_obj_Vdata;
	json_object *vehicle_jsonobject_list;
	char *transfer_str;
};
#endif
