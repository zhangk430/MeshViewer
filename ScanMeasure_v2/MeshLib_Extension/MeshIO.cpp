#include "MeshIO.h"

#pragma warning (disable : 4996)
//#pragma warning (disable : 4018)

bool MeshIO::ReadM(const char inputFile[], SimMesh & cMesh)
{	
	std::cout << "Reading Mesh " << inputFile << " ...\n";
	FILE * fp = fopen( inputFile, "r" );
	if( !fp ){
		std::cerr << "Can't open file " << inputFile << "!" <<std::endl;
		return false;
	}

	char line[1024];
	cMesh.clear();

	stdext::hash_map<int, int> vid2idx;

	while(!feof(fp)){	
		fgets(line, 1024, fp);
		if(!strlen(line)) continue;
		char * str = strtok( line, " \r\n");
		if (!str) continue;

		if( !strcmp(str, "Vertex" ) ){ //parsing a line of SimVertex element
			str = strtok(NULL," \r\n{");
			int id = atoi( str );
			SimVertex * v  = cMesh.createVertex(); // Important 1
			vid2idx[id] = v->idx;
			for( int i = 0 ; i < 3; i ++ ){
				str = strtok(NULL," \r\n{");
				v->p[i] = atof( str );
			}			
			//vertIndex[v] = v_adjInHEList.size();
			str = strtok( NULL, "\r\n");
			if(!str || strlen( str ) == 0) continue;
		}
		continue;
	}
	cMesh.init();
	fp = fopen( inputFile, "r" );
	while(!feof(fp)){	
		fgets(line, 1024, fp);
		if(!strlen(line)) continue;
		char * str = strtok( line, " \r\n");
		if (!str) continue;	
		if( !strcmp(str,"Face") ){ //parsing a line of SimFace element
			
			str = strtok(NULL, " \r\n");
			if( !str || strlen( str ) == 0 ) continue;
			int id = atoi( str );
			int vid[3];
			for( int i = 0; i < 3; i ++ )
			{
				str = strtok(NULL," \r\n{");
				vid[i] = vid2idx[atoi(str)];
			}
			SimFace * f = cMesh.createFace( vid );  //Important 2
			if (!str) continue;
			str = strtok( NULL, "\r\n");
			if( !str || strlen( str ) == 0 ) continue;
		}
	}

	fclose(fp);

	printf("Done!\n");
	return true;
}

bool MeshIO::ReadOBJ(const char inputFile[], SimMesh & cMesh)
{
	std::cout << "Reading Mesh " << inputFile << " ...\n";
	FILE * fp = fopen( inputFile, "r" );
	if( !fp ){
		std::cerr << "Can't open file " << inputFile << "!" <<std::endl;
		return false;
	}

	char line[1024];
	cMesh.clear();

	while(!feof(fp)){	
		fgets(line, 1024, fp);
		if(!strlen(line)) continue;
		char * str = strtok( line, " \r\n");
		if (!str) continue;

		if( !strcmp(str, "v" ) ){ //parsing a line of SimVertex element
			SimVertex * v  = cMesh.createVertex(); // Important 1
			for( int i = 0 ; i < 3; i ++ ){
				str = strtok(NULL," \r\n{");
				v->p[i] = atof( str );
			}			
		}
	}
	cMesh.init();

	fp = fopen( inputFile, "r" );
	while(!feof(fp)){	
		fgets(line, 1024, fp);
		if(!strlen(line)) continue;
		char * str = strtok( line, " \r\n");
		if (!str) continue;	
		if( !strcmp(str,"f") ){ //parsing a line of SimFace element
			int vid[3] = {-1, -1, -1};
			for( int i = 0; i < 3; i ++ )
			{
				str = strtok(NULL," \r\n{");
				if( !str || strlen( str ) == 0 ) continue;
				std::string v_id(str);
				int slash_pos = v_id.find_first_of('/');
				if (slash_pos >= 0)
					v_id = v_id.substr(0, slash_pos);
				vid[i] = atoi(v_id.c_str()) - 1;
			}
			if (vid[0] < 0)
				continue;
			SimFace * f = cMesh.createFace( vid );  //Important 2
			if (!str) continue;
		}
	}

	fclose(fp);

	printf("Done!\n");
	return true;
}

bool MeshIO::ReadOBJ(const char inputFile[], ModelView & modelView)
{
	std::cout << "Reading Mesh " << inputFile << " ...\n";
	FILE * fp = fopen( inputFile, "r" );
	if( !fp ){
		std::cerr << "Can't open file " << inputFile << "!" <<std::endl;
		return false;
	}

	char line[1024];
	modelView.theMesh->clear();
	std::vector<float> u, v;
	while(!feof(fp)){	
		fgets(line, 1024, fp);
		if(!strlen(line)) continue;
		char * str = strtok( line, " \r\n");
		if (!str) continue;

		if( !strcmp(str, "mtllib"))
		{
			std::string absolutePath(inputFile);
			int slash_pos = absolutePath.find_last_of('/');
			absolutePath = absolutePath.substr(0, slash_pos+1);
			modelView.theTexture = new MeshTexture(modelView.theMesh);
			str = strtok(NULL, "\r\n");
			std::string mtlFilename = absolutePath;
			mtlFilename.append(str);
			std::ifstream in(mtlFilename.c_str());
			if (in.is_open())
			{
				modelView.theTexture->mode = MeshTexture::CORNER_MODE;
				while (!in.eof())
				{
					std::string info;
					in >> info;
					if (!info.compare("map_Ka"))
					{
						std::string tFilename;
						in >> tFilename;
						modelView.theTexture->texture_filename = absolutePath;
						modelView.theTexture->texture_filename.append(tFilename);
						break;
					}
					else if (!info.compare("map_Ks"))
					{
						std::string tFilename;
						in >> tFilename;
						modelView.theTexture->texture_filename = absolutePath;
						modelView.theTexture->texture_filename.append(tFilename);
						break;
					}
					else if (!info.compare("map_Kd"))
					{
						std::string tFilename;
						in >> tFilename;
						modelView.theTexture->texture_filename = absolutePath;
						modelView.theTexture->texture_filename.append(tFilename);
						break;
					}
				}
			}
			in.close();
		}
		if( !strcmp(str, "v" ) ){ //parsing a line of SimVertex element
			SimVertex * v  = modelView.theMesh->createVertex(); // Important 1
			for( int i = 0 ; i < 3; i ++ ){
				str = strtok(NULL," \r\n{");
				v->p[i] = atof( str );
			}		
		}
		if( !strcmp(str, "vt" ) ){ //parsing a line of SimVertex element
			str = strtok(NULL," \r\n{");
			u.push_back(atof( str ));
			str = strtok(NULL," \r\n{");
			v.push_back(atof( str ));
		}
	}
	if (modelView.theTexture && (u.size() == 0 || modelView.theTexture->texture_filename == ""))
	{
		delete modelView.theTexture;
		modelView.theTexture = NULL;
	}
	modelView.theMesh->init();
	fp = fopen( inputFile, "r" );
	while(!feof(fp)){	
		fgets(line, 1024, fp);
		if(!strlen(line)) continue;
		char * str = strtok( line, " \r\n");
		if (!str) continue;	
		if( !strcmp(str,"f") ){ //parsing a line of SimFace element
			int vid[3] = {-1, -1, -1};
			for( int i = 0; i < 3; i ++ )
			{
				str = strtok(NULL," \r\n{");
				if( !str || strlen( str ) == 0 ) continue;
				std::string v_id(str);
				int slash_pos = v_id.find_first_of('/');
				if (slash_pos >= 0)
					v_id = v_id.substr(0, slash_pos);
				vid[i] = atoi(v_id.c_str()) - 1;
				if (modelView.theTexture)
				{
					std::string vt_id(str);
					vt_id = vt_id.substr(slash_pos+1);
					slash_pos = vt_id.find_first_of('/');
					if (slash_pos >= 0)
						vt_id = vt_id.substr(0, slash_pos);
					modelView.theTexture->u.push_back(u[atoi(vt_id.c_str())-1]);
					modelView.theTexture->v.push_back(v[atoi(vt_id.c_str())-1]);
				}
			}
			if (vid[0] < 0)
				continue;
			SimFace * f = modelView.theMesh->createFace( vid );  //Important 2
			if (!str) continue;
		}
	}

	fclose(fp);

	printf("Done!\n");
	return true;
}

bool MeshIO::ReadPLY(const char inputFile[], SimMesh & cMesh)
{
	std::cout << "Reading Mesh " << inputFile << " ...\n";
	
	std::ifstream in(inputFile);
	if (!in.is_open())
	{
		std::cerr << "Can't open file " << inputFile << "!" <<std::endl;
		return false;
	}
	int ver_num, face_num; 
	std::vector<std::string> property_list;
	bool end_header = false;
	bool end_vertex = false;
	bool end_face = false;
	bool end_vertex_property = false;
	while (!in.eof())
	{
		if (!end_header)
		{
			// read header infomation
			std::string info;
			in >> info;
			if (!info.compare("element"))
			{
				in >> info;
				std::string num;
				in >> num;
				if (!info.compare("vertex"))
					ver_num = atoi(num.c_str());
				else if (!info.compare("face"))
					face_num = atoi(num.c_str());
			}
			else if (!info.compare("property"))
			{
				std::string type;
				in >> type;
				if (!type.compare("list") && end_vertex_property)
				{
					end_vertex_property = true;
					continue;
				}
				std::string prop;
				in >> prop;
				property_list.push_back(prop);
			}
			else if (!info.compare("end_header"))
				end_header = true;
		}
		else if (!end_vertex)
		{
			if (cMesh.numVertices() < ver_num)
			{
				SimVertex * v  = cMesh.createVertex(); // Important 1		
				for (unsigned int i = 0; i < property_list.size(); i++)
				{
					if (!property_list[i].compare("x"))
						in >> v->p[0];
					else if (!property_list[i].compare("y"))
						in >> v->p[1];
					else if (!property_list[i].compare("z"))
						in >> v->p[2];
					else
					{
						float other;
						in >> other;
					}
				}
			}
			else
			{
				end_vertex = true;
				cMesh.init();
			}
		}
		else if (!end_face)
		{
			if (cMesh.numFaces() < face_num)
			{
				int vid[3];
				int v_num;
				in >> v_num;
				if (v_num != 3)
				{
					std::cout<<v_num<<std::endl;
					continue;
				}
				for( int i = 0; i < v_num; i ++ )
					in >> vid[i];
				SimFace * f = cMesh.createFace( vid );  //Important 2
			}
			else 
				end_face = true;
		}
		else
		{
			float other;
			in >> other;
		}
	}
	in.close();

	printf("Done!\n");
	return true;
}

bool MeshIO::ReadPLY(const char inputFile[], ModelView & modelView)
{
	std::cout << "Reading Mesh " << inputFile << " ...\n";

	std::ifstream in(inputFile);
	if (!in.is_open())
	{
		std::cerr << "Can't open file " << inputFile << "!" <<std::endl;
		return false;
	}
	modelView.theMesh->clear();
	int ver_num, face_num; 
	std::vector<std::string> property_list;
	bool end_header = false;
	bool end_vertex = false;
	bool end_face = false;
	modelView.theNormals = new MeshNormal(modelView.theMesh);
	modelView.theColor = new MeshColor(modelView.theMesh);
	std::string line;
	bool end_vertex_property = false;
	while (std::getline(in, line))
	{
		std::istringstream iss(line);
		if (!end_header)
		{
			// read header infomation
			std::string info;
			while(iss >> info)
			{
				if (!info.compare("element"))
				{
					iss >> info;
					std::string num;
					iss >> num;
					if (!info.compare("vertex"))
						ver_num = atoi(num.c_str());
					else if (!info.compare("face"))
						face_num = atoi(num.c_str());
				}
				else if (!info.compare("property") && !end_vertex_property)
				{
					std::string type;
					iss >> type;
					if (!type.compare("list"))
					{
						end_vertex_property = true;
						continue;
					}
					std::string prop;
					iss >> prop;
					property_list.push_back(prop);
				}
				else if (!info.compare("end_header"))
					end_header = true;
			}
			
		}
		else if (!end_vertex)
		{
			if (modelView.theMesh->numVertices() < ver_num)
			{
				SimVertex * v  = modelView.theMesh->createVertex(); // Important 1	
				Point n;
				int color;
				for (unsigned int i = 0; i < property_list.size(); i++)
				{
					if (!property_list[i].compare("x"))
						iss >> v->p[0];
					else if (!property_list[i].compare("y"))
						iss >> v->p[1];
					else if (!property_list[i].compare("z"))
						iss >> v->p[2];
					else if (!property_list[i].compare("nx"))
						iss >> n[0];
					else if (!property_list[i].compare("ny"))
						iss >> n[1];
					else if (!property_list[i].compare("nz"))
					{
						iss >> n[2];
						n /= n.norm();
						modelView.theNormals->vNormals.push_back(n);
					}
					else if (!property_list[i].compare("red"))
					{
						iss >> color;
						modelView.theColor->r.push_back(static_cast<unsigned char> (color));
					}
					else if (!property_list[i].compare("green"))
					{
						iss >> color;
						modelView.theColor->g.push_back(static_cast<unsigned char> (color));
					}
					else if (!property_list[i].compare("blue"))
					{
						iss >> color;
						modelView.theColor->b.push_back(static_cast<unsigned char> (color));
					}
					else if (!property_list[i].compare("alpha"))
					{
						iss >> color;
						modelView.theColor->a.push_back(static_cast<unsigned char> (color));
					}
					else
					{
						float other;
						iss >> other;
					}
				}
			}
			else
			{
				end_vertex = true;
				modelView.theMesh->init();
			}
		}
		else if (!end_face)
		{
			if (modelView.theMesh->numFaces() < face_num)
			{
				int vid[3];
				int v_num;
				iss >> v_num;
				if (v_num != 3)
				{
					std::cout << "Not Triangular Mesh!" << std::endl;
					continue;
				}
				for( int i = 0; i < v_num; i ++ )
					iss >> vid[i];
				SimFace * f = modelView.theMesh->createFace( vid );  //Important 2
			}
			else 
				end_face = true;
		}
		/*else
		{
			float other;
			in >> other;
		}*/
	}

	in.close();

	if (modelView.theColor->r.size() != modelView.theMesh->numVertices())
	{
		delete modelView.theColor;
		modelView.theColor = NULL;
	}

	printf("Done!\n");
	return true;
}

bool MeshIO::ReadPT(const char inputFile[], ModelView & modelView)
{
	std::ifstream in(inputFile);
	if (!in.is_open())
	{
		std::cerr << "Cannot open file " << inputFile << "to read!" <<std::endl;
		return false;
	}
	modelView.theMesh->clear();
	modelView.theColor = new MeshColor(modelView.theMesh);
	std::string line;
	while (std::getline(in, line))
	{
		if (line.size() == 0)
			break;
		std::istringstream iss(line);
		SimVertex * v = modelView.theMesh->createVertex();
		iss >> v->p[0] >> v->p[1] >> v->p[2];
		int color;
		iss >> color;
		modelView.theColor->r.push_back(static_cast<unsigned char> (color));
		iss >> color;
		modelView.theColor->g.push_back(static_cast<unsigned char> (color));		
		iss >> color;
		modelView.theColor->b.push_back(static_cast<unsigned char> (color));
	}
	in.close();
	printf("Done!\n");
	return true;
}

bool MeshIO::WriteM( const char outputFile[], SimMesh & cMesh )
{
	FILE * fp = fopen( outputFile,"w");
	if ( !fp ){
		std::cerr << "Cannot open file " << outputFile << "to write!" <<std::endl;
		return false;
	}

	std::cout << "Writing Mesh "<< outputFile <<" ...";
	for (int i = 0; i <cMesh.numVertices(); i++){
		SimVertex * ver = cMesh.indVertex(i);
		std::ostringstream oss;
		oss.precision(10); oss.setf(std::ios::fixed,std::ios::floatfield);  //setting the output precision: now 10
		oss << "Vertex " << ver->idx + 1 << " " << ver->p[0] << " " << ver->p[1] << " " << ver->p[2];
		fprintf(fp, "%s\n", oss.str().c_str());
	}

	std::vector<SimFace *>::iterator fit;
	for (int i = 0; i < cMesh.numFaces(); i++)
	{
		SimFace * f = cMesh.indFace(i);
		int v0 = f->ver[0]->idx;
		int v1 = f->ver[1]->idx;
		int v2 = f->ver[2]->idx;
		fprintf(fp, "Face %d %d %d %d\n", f->idx+1, v0+1, v1+1, v2+1);
	}
	fclose(fp);
	std::cout << "Done!" <<std::endl;
	return true;
}

bool MeshIO::WritePT(const char outputFile[], ModelView & modelView)
{
	SimMesh * mesh = modelView.theMesh;
	MeshColor * color = modelView.theColor;
	if (!mesh)
		return false;
	std::ofstream out(outputFile);
	for (int i = 0; i < mesh->numVertices(); i++)
	{
		SimVertex * v = mesh->indVertex(i);
		out << v->p[0] << " " << v->p[1] << " " << v->p[2] << " ";
		if (color)
			out << static_cast<int> (color->r[i]) << " " << static_cast<int> (color->g[i]) << " " << static_cast<int> (color->b[i]);
		out << "\n";
	}
	out.close();
	return true;
}