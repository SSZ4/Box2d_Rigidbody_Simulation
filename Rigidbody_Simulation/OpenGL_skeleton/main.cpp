#include<stdio.h>
#include<stdlib.h>
#include<glut.h>
#include<set>
#include<vector>
#include<Box2D/Box2D.h>

#define RADTODEG b2_pi/180
using namespace std;
typedef pair<b2Fixture*, b2Fixture*> fixturePair;


float PI = 3.141592653589793238;
int scr_width = 600;
int scr_height = 600;
float lookatx = 0;
b2World* world;
b2Body* box[42];
b2PolygonShape boxshape[42];
b2Body* ground[4];
b2EdgeShape gnd_shape[4];
b2Body* playerbox;
b2CircleShape playerboxshape;

b2PulleyJoint* m_joint;
b2DistanceJoint* d_joint[6];
b2Body* body1 = NULL;
b2Body* body2 = NULL;
b2Body* djbody[6];
b2PolygonShape jboxshape;
b2PolygonShape djboxshape[6];
int32 velocityIterations = 8;
int32 positionIterations = 3;
float32 g_hz = 60.0f;
float32 timeStep = 1.0f / g_hz;
set<fixturePair> m_fixturePairs;

class b2ContactListener_ : public b2ContactListener
{
public:

	b2ContactListener_() {};

	void BeginContact(b2Contact* contact)
	{
		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();

		if (fixtureA->IsSensor() && fixtureB->GetBody()->GetType() == b2_dynamicBody)
			m_fixturePairs.insert(make_pair(fixtureA, fixtureB));
		else if (fixtureB->IsSensor() && fixtureA->GetBody()->GetType() == b2_dynamicBody)
			m_fixturePairs.insert(make_pair(fixtureB, fixtureA));
	}

	void EndContact(b2Contact* contact)
	{
		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();

		if (fixtureA->IsSensor() && fixtureB->GetBody()->GetType() == b2_dynamicBody)
			m_fixturePairs.erase(make_pair(fixtureA, fixtureB));
		else if (fixtureB->IsSensor() && fixtureA->GetBody()->GetType() == b2_dynamicBody)
			m_fixturePairs.erase(make_pair(fixtureB, fixtureA));
	}

	void PreSolve(b2Contact* contact) {}
	void PostSolve(b2Contact* contact) {}

};


b2ContactListener_ contactListener; //= b2ContactListener_();

//Function declaraions
void Render();
void Keyboard(unsigned char k, int x, int y);
void Reshape(int w, int h);
void Update(int value);
void Setup();
void SetupBoxbdst(float x, float y, float boxw, float boxh, int angle, int nary);
void CreateBoxbdst(int nary);
void SetupBoxbddy(float x, float y, float boxw, float boxh, int angle, int nary, bool is_gravity = false, bool is_kinematic = false, bool is_first = false);
void CreateBoxbddy(int nary);
void SetupGround(float x1, float y1, float x2, float y2, int nary);
void CreateGround(int nary);
void SetupPlayerBall();
void CreatePlayerBall();
void SetupJoint1();
void CreateJoint1();
void SetupJoint2(int x, int y, float box_w, float box_h, int nary);
void CreateJoint2(int nary);
void SetupWater(float x, float y, float boxw, float boxh, int angle, int nary);
void CreateWater(int nary);
void applydrag(b2Fixture* box, b2Fixture* water, float area, b2Vec2 centroid);
void applybuoyancy(b2Fixture* box, b2Fixture* water, float area, b2Vec2 gravity, b2Vec2 centroid);
b2Vec2 ComputeCentroid(vector<b2Vec2> vs, float& area);
b2Vec2 intersection(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 s, b2Vec2 e);
bool inside(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 p);
bool findIntersectionOfFixtures(b2Fixture* fA, b2Fixture* fB, vector<b2Vec2>& outputVertices);


//------------------------------------------------------------------------------
int main(int argc, char* argv[]) {

	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(scr_width, scr_height);
	glutInit(&argc, argv);
	glutCreateWindow("Box2D");

	Setup();

	glutDisplayFunc(Render);
	glutReshapeFunc(Reshape);
	glutTimerFunc(20, Update, 0);
	glutKeyboardFunc(Keyboard);
	glutMainLoop();

	return 0;
}
//------------------------------------------------------------------------------
void Keyboard(unsigned char key, int x, int y)
{
	//int x_force = 200;
	//int y_force = 1800;
	switch (key) { //wake가 false면 정지 상태에서는 힘이 전달 안된다. 주의
	case 'a':
		if (playerbox->GetLinearVelocity().x > -20.0f)
			playerbox->SetLinearVelocity(b2Vec2(-40.0f, playerbox->GetLinearVelocity().y));
		else
			break;
		//playerbox->ApplyLinearImpulse(b2Vec2(-250.0f, 0.0f), playerbox->GetWorldCenter(), true);
		//playerbox->SetLinearVelocity(b2Vec2(-40.0f, playerbox->GetLinearVelocity().y));
		//playerbox->ApplyForce(b2Vec2(-x_force, 0), playerbox->GetWorldCenter(), true);
		break;
	case 'd':
		if (playerbox->GetLinearVelocity().x < 20.0f)
			playerbox->SetLinearVelocity(b2Vec2(40.0f, playerbox->GetLinearVelocity().y));
		else
			break;
		//playerbox->ApplyLinearImpulse(b2Vec2(250.0f, 0.0f), playerbox->GetWorldCenter(), true);
		//playerbox->SetLinearVelocity(b2Vec2(40.0f, playerbox->GetLinearVelocity().y));
		//playerbox->ApplyForce(b2Vec2(x_force, 0), playerbox->GetWorldCenter(), true);
		break;
	case 'w':
		if (playerbox->GetLinearVelocity().y < 10.0f)
			playerbox->SetLinearVelocity(b2Vec2(playerbox->GetLinearVelocity().x, 60.0f));
		else
			break;
		//playerbox->ApplyLinearImpulse(b2Vec2(0, 500.0f), playerbox->GetWorldCenter(), true);
		//playerbox->SetLinearVelocity(b2Vec2(playerbox->GetLinearVelocity().x, 60.0f));
		//playerbox->ApplyForce(b2Vec2(0, y_force), playerbox->GetWorldCenter(), true);
		break;
	case 's':
		if (playerbox->GetLinearVelocity().y > 30.0f)
			playerbox->SetLinearVelocity(b2Vec2(playerbox->GetLinearVelocity().x, -40.0f));
		else
			break;
		//playerbox->ApplyLinearImpulse(b2Vec2(0, -500.0f), playerbox->GetWorldCenter(), true);
		//playerbox->SetLinearVelocity(b2Vec2(playerbox->GetLinearVelocity().x, -45.0f));
		//playerbox->ApplyForce(b2Vec2(0, -y_force/3), playerbox->GetWorldCenter(), true);
		break;
	case ' ':
		playerbox->SetLinearVelocity(b2Vec2(0.0f, 0.0f));
		break;
	case 'e':
		lookatx += 3.0f;
		break;
	case 'q':
		lookatx -= 3.0f;
		break;
	default:
		break;
	}
	glutPostRedisplay();
}
//-----------------------------------------------------------------------------

void Update(int value) {
	b2ContactListener_ listener;// = b2ContactListener_();
	world->SetContactListener(&listener);
	world->Step(timeStep, velocityIterations, positionIterations);

	if (m_fixturePairs.size() > 0) {
		set<fixturePair>::iterator it = m_fixturePairs.begin();
		set<fixturePair>::iterator end = m_fixturePairs.end();

		while (it != end) {

			b2Fixture* fixtureA = it->first;
			b2Fixture* fixtureB = it->second;

			float density = fixtureA->GetDensity();

			vector<b2Vec2> intersectionPoints;
			if (findIntersectionOfFixtures(fixtureB, fixtureA, intersectionPoints)) {
				//printf("hello");
				//find centroid
				float area = 0;
				b2Vec2 centroid = ComputeCentroid(intersectionPoints, area);
				applybuoyancy(fixtureB, fixtureA, area, b2Vec2(0.0f, -100.0f), centroid); //box->a water->b
				applydrag(fixtureB, fixtureA, area, centroid);
			}
			++it;

		}
	}

	glutPostRedisplay();
	glutTimerFunc(20, Update, 0);
}
void Reshape(int w, int h)
{
	scr_width = w;
	scr_height = h;

	glViewport(0, 0, scr_width, scr_height);

	glutPostRedisplay();
}
//------------------------------------------------------------------------------
void Render()
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	b2Vec2 position = playerbox->GetPosition();
	b2Vec2 poss = position;

	gluPerspective(60.0, 600.0 / 600.0, 1.0, 120.0);
	glTranslatef(-poss.x, 0, 0);
	gluLookAt(lookatx, 0, 120, lookatx, 0, 0, 0, 1, 0);
	glMatrixMode(GL_MODELVIEW);

	CreatePlayerBall();

	CreateGround(0); //배경윗줄
	CreateGround(1); //배경아랫줄
	CreateGround(2); //배경왼쪽줄	
	CreateGround(3); //배경오른쪽줄	

	CreateBoxbdst(0);
	CreateBoxbdst(1);
	CreateBoxbdst(2);
	CreateBoxbdst(3);
	CreateBoxbdst(4);
	CreateBoxbdst(5);
	CreateBoxbdst(6);
	CreateBoxbdst(7);
	CreateBoxbdst(8); //대각선 그거
	CreateBoxbdst(9); //대각선 위에 그거
	CreateBoxbdst(10);
	CreateBoxbdst(11);
	CreateBoxbdst(12);
	CreateBoxbdst(13);
	CreateBoxbdst(14);


	CreateBoxbddy(15);
	CreateBoxbddy(16);
	CreateBoxbddy(17);
	CreateBoxbddy(18);
	CreateBoxbddy(19);
	CreateBoxbddy(20);
	CreateBoxbddy(41);


	CreateBoxbddy(21);

	CreateBoxbdst(22);
	CreateBoxbdst(23);
	CreateBoxbdst(24);
	CreateBoxbdst(25);
	CreateBoxbdst(26);
	CreateBoxbdst(27);
	CreateBoxbdst(28);
	CreateBoxbdst(29);
	CreateBoxbdst(30);
	CreateBoxbdst(31);
	CreateBoxbdst(32);
	CreateBoxbdst(33);
	CreateBoxbdst(34);
	CreateBoxbdst(35);
	CreateBoxbdst(36);
	CreateBoxbdst(37);

	CreateBoxbddy(38);
	CreateWater(39);
	CreateWater(40);

	CreateJoint1();
	CreateJoint2(0);
	CreateJoint2(1);
	CreateJoint2(2);



	//glFlush();
	glutSwapBuffers();
}
//------------------------------------------------------------------------------
void Setup() {

	b2Vec2 gravity;
	gravity.Set(0.0f, -100.0f);
	world = new b2World(gravity);

	SetupPlayerBall();
	SetupGround(-50.0f, -68.0f, 520.0f, -68.0f, 0); //배경윗줄
	SetupGround(-50.0f, 68.0f, 520.0f, 68.0f, 1); //배경아랫줄
	SetupGround(-46.0f, -68.0f, -46.0f, 68.0f, 2); //배경왼쪽줄	
	SetupGround(520.0f, -68.0f, 520.0f, 68.0f, 3); //배경오른쪽줄

	SetupBoxbdst(-20.0f, -63.0f, 3.0f, 5.0f, 0.0f, 0);
	SetupBoxbdst(-5.0f, -58.0f, 3.5f, 10.0f, 0.0f, 1);
	SetupBoxbdst(10.0f, -53.0f, 4.5f, 15.0f, 0.0f, 2);
	SetupBoxbdst(30.0f, -48.0f, 5.0f, 20.0f, 0.0f, 3);

	SetupBoxbddy(12.5f, 5.0f, 12.0f, 2.0f, 0.0f, 4, false, false, true);//456 joint넣어야댐 ㅇㅇ;
	SetupBoxbddy(-10.0f, -10.5f, 8.0f, 2.0f, 0.0f, 5, false, false, true);
	SetupBoxbddy(-35.0f, 0.0f, 7.0f, 2.0f, 0.0f, 6, false, false, true);


	SetupBoxbdst(3.5f, 30.0f, 25.0f, 2.0f, 0.0f, 7);
	SetupBoxbdst(60.0f, -18.0f, 60.0f, 2.0f, -65.0f * PI / 180, 8); //대각선 그거
	SetupBoxbdst(59.5f, 5.0f, 55.0f, 2.0f, -65.0f * PI / 180, 9); //대각선 위에 그거 1스테 끝

	SetupBoxbdst(100.0f, 30.0f, 4.0f, 40.0f, 0.0f, 10);
	SetupBoxbdst(120.0f, -6.0f, 20.0f, 4.0f, 0.0f, 11);
	SetupBoxbdst(150.0f, 31.0f, 20.0f, 4.0f, 0.0f, 12);
	SetupBoxbdst(170.0f, -17.5f, 4.0f, 52.5f, 0.0f, 13);
	SetupBoxbdst(157.0f, -40.0f, 12.5f, 4.0f, 0.0f, 14);



	SetupBoxbddy(109.0f, -19.0f, 3.0f, 3.0f, 0.0f, 41);
	SetupBoxbddy(109.0f, -25.0f, 3.0f, 3.0f, 0.0f, 15);
	SetupBoxbddy(109.0f, -31.0f, 3.0f, 3.0f, 0.0f, 16);
	SetupBoxbddy(109.0f, -37.0f, 3.0f, 3.0f, 0.0f, 17);
	SetupBoxbddy(109.0f, -43.0f, 3.0f, 3.0f, 0.0f, 18);
	SetupBoxbddy(109.0f, -48.0f, 3.0f, 3.0f, 0.0f, 19);
	SetupBoxbddy(109.0f, -55.0f, 3.0f, 3.0f, 0.0f, 20);


	SetupBoxbddy(169.0f, 50.0f, 3.5f, 15.0f, 0.0f, 21);//2stage finish

	SetupBoxbdst(240.0f, -17.5f, 3.0f, 52.5f, 0.0f, 22); //170과 240사이에 물있음
	SetupBoxbdst(205.0f, -37.5f, 35.0f, 30.0f, 0.0f, 23);
	SetupBoxbddy(200.0f, 30.0f, 6.0f, 6.0f, 0.0f, 24);
	SetupBoxbddy(220.0f, 28.0f, 4.0f, 5.0f, 0.0f, 25);

	SetupBoxbdst(322.7f, -35.0f, 40.0f, 1.3f, 0.0f, 26);
	SetupBoxbdst(285.0f, -45.0f, 2.0f, 10.0f, 0.0f, 27);
	SetupBoxbdst(300.0f, -58.0f, 2.0f, 10.0f, 0.0f, 28);
	SetupBoxbdst(315.0f, -45.0f, 2.0f, 10.0f, 0.0f, 29);
	SetupBoxbdst(330.0f, -58.0f, 2.0f, 10.0f, 0.0f, 30);

	SetupBoxbdst(288.0f, 43.0f, 3.0f, 25.0f, 0.0f, 31);
	SetupBoxbdst(322.5f, -20.0f, 22.0f, 13.0f, 0.0f, 32);
	SetupJoint2(315.0f, 20.0f, 8.0f, 2.0f, 0);
	SetupBoxbdst(340.0f, 25.0f, 5.0f, 20.0f, 0.0f, 33);
	SetupBoxbdst(360.0f, 20.5f, 3.0f, 50.5f, 0.0f, 34);//+15, 3stage finish

	SetupBoxbdst(365.0f, -11.0f, 8.0f, 25.0f, 0.0f, 35);
	SetupBoxbdst(410.0f, -16.5f, 3.0f, 52.5f, 0.0f, 36);
	SetupBoxbdst(513.0f, -37.3f, 6.0f, 32.0f, 0.0f, 37);
	SetupJoint2(440.0f, 0.0f, 12.0f, 2.0f, 1);
	SetupJoint2(480.0f, 13.0f, 10.0f, 2.0f, 2);

	SetupBoxbddy(389.5f, -5.0f, 9.0f, 2.0f, 0.0f, 38, false, true);

	SetupJoint1();//233, 290

	//SetupWater(-25.0f, -20.0f, 10.0f, 5.0f, 0.0f, 39);
	SetupWater(205.0f, 5.0f, 32.0f, 15.0f, 0.0f, 39);
	SetupWater(460.0f, -39.0f, 47.0f, 30.0f, 0.0f, 40);




}
//------------------------------------------------------------------------------








void SetupBoxbdst(float x, float y, float boxw, float boxh, int angle, int nary) {
	b2BodyDef boxbd;
	boxbd.type = b2_staticBody;
	boxbd.position.Set(x, y);
	boxbd.angle = angle;
	b2Body* body = world->CreateBody(&boxbd);
	boxshape[nary].SetAsBox(boxw, boxh);
	b2FixtureDef boxfd;
	boxfd.shape = &boxshape[nary];
	boxfd.density = 1.0f; //일반적으로 밀도 = 1
	boxfd.restitution = 0.3f;
	boxfd.friction = 0.3f;
	body->CreateFixture(&boxfd);
	box[nary] = body;
	//box->SetLinearVelocity(b2Vec2(0.0f, 10.0f));

}
void CreateBoxbdst(int nary) {
	b2Vec2 pos = box[nary]->GetPosition();
	float32 ang = box[nary]->GetAngle();
	glPushMatrix();
	glTranslatef(pos.x, pos.y, 0.0f);
	glRotatef(ang * 180 / PI, 0.0f, 0.0f, 1.0f);
	glColor3f(0.3f, 0.7f, 0.4f);
	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++)
		glVertex2f(boxshape[nary].m_vertices[i].x, boxshape[nary].m_vertices[i].y);
	glEnd();
	glPopMatrix();
}
//------------------------------------------------------------------------------
void SetupBoxbddy(float x, float y, float boxw, float boxh, int angle, int nary, bool is_gravity, bool is_kinematic, bool is_first) {
	b2BodyDef boxbd;
	if (!is_kinematic)
		boxbd.type = b2_dynamicBody;
	else
		boxbd.type = b2_kinematicBody;
	boxbd.position.Set(x, y);
	boxbd.angle = angle;
	b2Body* body = world->CreateBody(&boxbd);
	boxshape[nary].SetAsBox(boxw, boxh);
	b2FixtureDef boxfd;
	boxfd.shape = &boxshape[nary];
	boxfd.density = 0.1f; //일반적으로 밀도 = 1
	boxfd.restitution = 0.001f;
	boxfd.friction = 1.0f;	
	if (is_gravity)
		body->SetGravityScale(0);
	if (is_kinematic)
		body->SetAngularVelocity(5.0f);
	if (is_first) {
		boxfd.density = 0.5f; //일반적으로 밀도 = 1
		boxfd.restitution = 0.1f;
		boxfd.friction = 1.0f;
		body->SetLinearDamping(INFINITY);
		body->SetAngularDamping(5.0f);
	}
	body->CreateFixture(&boxfd);
	box[nary] = body;
	body->SetAngularDamping(0.2f);
	//body->SetLinearDamping(0.5f);
}
void CreateBoxbddy(int nary) {
	b2Vec2 pos = box[nary]->GetPosition();
	float32 ang = box[nary]->GetAngle();
	glPushMatrix();
	glTranslatef(pos.x, pos.y, 0.0f);
	glRotatef(ang * 180 / PI, 0.0f, 0.0f, 1.0f);
	glColor3f(0.7f, 0.4f, 0.3f);
	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++)
		glVertex2f(boxshape[nary].m_vertices[i].x, boxshape[nary].m_vertices[i].y);
	glEnd();
	glPopMatrix();
}
//------------------------------------------------------------------------------
void SetupPlayerBall() {

	b2BodyDef boxbd;
	boxbd.type = b2_dynamicBody;
	boxbd.position.Set(-40.0f, -40.0f);
	b2Body* body = world->CreateBody(&boxbd);
	playerboxshape.m_radius = 1.5f;
	b2FixtureDef boxfd;
	boxfd.shape = &playerboxshape;
	boxfd.density = 1.0f; //일반적으로 밀도 = 1
	boxfd.restitution = 0.01f;
	boxfd.friction = 10.0f;
	body->CreateFixture(&boxfd);
	playerbox = body;
	//playerbox->SetFixedRotation(true);
	//playerbox->SetLinearDamping(1.0f);
	//playerbox->SetLinearVelocity(b2Vec2(0.0f, 10.0f));
}

void CreatePlayerBall() {
	b2Vec2 pos = playerbox->GetPosition();
	float32 ang = playerbox->GetAngle();
	glPushMatrix();
	glTranslatef(pos.x, pos.y, 0.0f);
	glRotatef(ang * 180 / PI, 0.0f, 0.0f, 1.0f);
	glColor3f(1.0f, 1.0f, 1.0f);

	glBegin(GL_POLYGON);
	for (int i = 0; i < 360; i++)
	{
		float deg2Rad = i * PI / 180;
		glVertex2f(cos(deg2Rad) * playerboxshape.m_radius, sin(deg2Rad) * playerboxshape.m_radius);
	}
	glEnd();
	glPopMatrix();
}
//------------------------------------------------------------------------------
void SetupGround(float x1, float y1, float x2, float y2, int nary) {
	b2BodyDef gnd_bd;
	ground[nary] = world->CreateBody(&gnd_bd);
	gnd_shape[nary].Set(b2Vec2(x1, y1), b2Vec2(x2, y2));
	ground[nary]->CreateFixture(&gnd_shape[nary], 0.0f);
}

void CreateGround(int nary) {
	b2Vec2 position = ground[nary]->GetPosition();
	float32 angle = ground[nary]->GetAngle();

	glPushMatrix();
	glTranslatef(position.x, position.y, 0.0f);
	glRotatef(angle, 0.0f, 0.0f, 1.0f);
	glColor3f(0.3f, 0.7f, 0.4f);

	glLineWidth(5.0f);
	glBegin(GL_LINES);
	glVertex2d(gnd_shape[nary].m_vertex1.x, gnd_shape[nary].m_vertex1.y);
	glVertex2d(gnd_shape[nary].m_vertex2.x, gnd_shape[nary].m_vertex2.y);
	glEnd();
	glPopMatrix();
}

void SetupJoint1() {
	b2Vec2 groundanchor1, groundanchor2;
	b2Vec2 anchor1, anchor2;

	groundanchor1.Set(276.0f, 65.0f);
	groundanchor2.Set(350.0f, 65.0f);
	float st_length1 = 115.0f;
	float st_length2 = 50.0f;
	float32 box_w = 5.0f;
	float32 box_h = 8.0f;
	jboxshape.SetAsBox(box_w, box_h);

	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(groundanchor1.x, groundanchor1.y - st_length1 - box_h);
	body1 = world->CreateBody(&bd);
	body1->CreateFixture(&jboxshape, 0.5f);

	bd.position.Set(groundanchor2.x, groundanchor2.y - st_length2 - box_h);
	body2 = world->CreateBody(&bd);
	body2->CreateFixture(&jboxshape, 0.5f);

	float ratio = 1.0f;
	b2PulleyJointDef pulleyDef;
	anchor1.Set(groundanchor1.x, groundanchor1.y - st_length1);
	anchor2.Set(groundanchor2.x, groundanchor2.y - st_length2);
	pulleyDef.Initialize(body1, body2, groundanchor1, groundanchor2, anchor1, anchor2, ratio);
	m_joint = (b2PulleyJoint*)world->CreateJoint(&pulleyDef);
}
void CreateJoint1() {
	b2Vec2 pos1 = body1->GetPosition();
	float a1 = body1->GetAngle();
	glPushMatrix();
	glTranslatef(pos1.x, pos1.y, 0.0f);
	glRotatef(a1 * 180 / PI, 0.0f, 0.0f, 1.0f);
	glColor3f(0.1f, 0.8f, 0.2f);
	glLineWidth(1.0f);
	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(jboxshape.m_vertices[i].x, jboxshape.m_vertices[i].y);
	}
	glEnd();
	glPopMatrix();

	b2Vec2 pos2 = body2->GetPosition();
	float a2 = body2->GetAngle();
	glPushMatrix();
	glTranslatef(pos2.x, pos2.y, 0.0f);
	glRotatef(a2, 0.0f, 0.0f, 1.0f);
	glColor3f(0.1f, 0.8f, 0.2f);
	glLineWidth(1.0f);
	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(jboxshape.m_vertices[i].x, jboxshape.m_vertices[i].y);
	}
	glEnd();
	glPopMatrix();

	glPushMatrix();
	glColor3f(0.8f, 0.8f, 0.8f);
	glLineWidth(1.0f);
	glBegin(GL_LINE_STRIP);
	glVertex2d(m_joint->GetAnchorA().x, m_joint->GetAnchorA().y);
	glVertex2d(m_joint->GetGroundAnchorA().x, m_joint->GetGroundAnchorA().y);
	glVertex2d(m_joint->GetGroundAnchorB().x, m_joint->GetGroundAnchorB().y);
	glVertex2d(m_joint->GetAnchorB().x, m_joint->GetAnchorB().y);
	glEnd();
	glPopMatrix();
}

void SetupJoint2(int x, int y, float box_w, float box_h, int nary) {
	//float st_length = 30.0f;
	float32 box_w1 = box_w;
	float32 box_h1 = box_h;
	float32 box_w2 = 0.1f;
	float32 box_h2 = 0.1f;
	b2Vec2 anchor1, anchor2, anchor3;
	djboxshape[2 * nary].SetAsBox(box_w1, box_h1);
	djboxshape[2 * nary + 1].SetAsBox(box_w2, box_h2);

	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(x, y);
	djbody[2 * nary] = world->CreateBody(&bd);
	djbody[2 * nary]->CreateFixture(&djboxshape[2 * nary], 0.5f);
	djbody[2 * nary]->SetLinearDamping(1.0f);

	b2BodyDef bd2;
	bd2.type = b2_staticBody;
	bd2.position.Set(x, y + 15.0f);
	djbody[2 * nary + 1] = world->CreateBody(&bd2);
	djbody[2 * nary + 1]->CreateFixture(&djboxshape[2 * nary + 1], 0.5f);

	b2DistanceJointDef DistDef;
	anchor1.Set(djboxshape[2 * nary].m_vertices[0].x + djbody[2 * nary]->GetPosition().x, djboxshape[2 * nary].m_vertices[0].y + djbody[2 * nary]->GetPosition().y);
	anchor2.Set(djboxshape[2 * nary].m_vertices[1].x + djbody[2 * nary]->GetPosition().x, djboxshape[2 * nary].m_vertices[1].y + djbody[2 * nary]->GetPosition().y);
	anchor3.Set(djbody[2 * nary + 1]->GetPosition().x, djbody[2 * nary + 1]->GetPosition().y);
	DistDef.Initialize(djbody[2 * nary], djbody[2 * nary + 1], anchor1, anchor3);
	DistDef.collideConnected = true;
	d_joint[2 * nary] = (b2DistanceJoint*)world->CreateJoint(&DistDef);

	b2DistanceJointDef DistDef2;
	DistDef2.Initialize(djbody[2 * nary], djbody[2 * nary + 1], anchor2, anchor3);
	DistDef.collideConnected = true;
	d_joint[2 * nary + 1] = (b2DistanceJoint*)world->CreateJoint(&DistDef2);

}

void CreateJoint2(int nary) {
	b2Vec2 pos1 = djbody[2 * nary]->GetPosition();
	float a1 = djbody[2 * nary]->GetAngle();
	glPushMatrix();
	glTranslatef(pos1.x, pos1.y, 0.0f);
	glRotatef(a1 * 180 / PI, 0.0f, 0.0f, 1.0f);
	glColor3f(0.1f, 0.8f, 0.2f);
	glLineWidth(1.0f);
	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(djboxshape[2 * nary].m_vertices[i].x, djboxshape[2 * nary].m_vertices[i].y);
	}
	glEnd();
	glPopMatrix();

	b2Vec2 pos2 = djbody[2 * nary + 1]->GetPosition();
	float a2 = djbody[2 * nary + 1]->GetAngle();
	glPushMatrix();
	glTranslatef(pos2.x, pos2.y, 0.0f);
	glRotatef(a2, 0.0f, 0.0f, 1.0f);
	glColor3f(0.1f, 0.8f, 0.2f);
	glLineWidth(1.0f);
	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(djboxshape[2 * nary + 1].m_vertices[i].x, djboxshape[2 * nary + 1].m_vertices[i].y);
	}
	glEnd();
	glPopMatrix();

	glPushMatrix();
	glColor3f(0.8f, 0.8f, 0.8f);
	glLineWidth(1.0f);
	glBegin(GL_LINES);
	glVertex2d(d_joint[2 * nary]->GetAnchorA().x, d_joint[2 * nary]->GetAnchorA().y);
	glVertex2d(d_joint[2 * nary]->GetAnchorB().x, d_joint[2 * nary]->GetAnchorB().y);
	glVertex2d(d_joint[2 * nary + 1]->GetAnchorA().x, d_joint[2 * nary + 1]->GetAnchorA().y);
	glVertex2d(d_joint[2 * nary + 1]->GetAnchorB().x, d_joint[2 * nary + 1]->GetAnchorB().y);
	glEnd();
	glPopMatrix();
}


void CreateWater(int nary) {
	b2Vec2 pos = box[nary]->GetPosition();
	float32 ang = box[nary]->GetAngle();
	glPushMatrix();
	glTranslatef(pos.x, pos.y, 0.0f);
	glRotatef(ang * 180 / PI, 0.0f, 0.0f, 1.0f);
	glColor3f(0.0f, 0.0f, 1.0f);
	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++)
		glVertex2f(boxshape[nary].m_vertices[i].x, boxshape[nary].m_vertices[i].y);
	glEnd();
	glPopMatrix();
}
void SetupWater(float x, float y, float boxw, float boxh, int angle, int nary) {
	b2BodyDef boxbd;
	boxbd.type = b2_staticBody;
	boxbd.position.Set(x, y);
	boxbd.angle = angle;
	b2Body* body = world->CreateBody(&boxbd);
	boxshape[nary].SetAsBox(boxw, boxh);
	b2FixtureDef boxfd;
	boxfd.shape = &boxshape[nary];
	boxfd.density = 4.0f; //일반적으로 밀도 = 1
	boxfd.restitution = 0.0f;
	boxfd.friction = 5.0f;
	boxfd.isSensor = true;
	body->CreateFixture(&boxfd);
	box[nary] = body;
}



bool findIntersectionOfFixtures(b2Fixture* fA, b2Fixture* fB, vector<b2Vec2>& outputVertices)
{
	//currently this only handles polygon vs polygon

	if (fA->GetShape()->GetType() != b2Shape::e_polygon || fB->GetShape()->GetType() != b2Shape::e_polygon)
		return false;


	b2PolygonShape* polyA = (b2PolygonShape*)fA->GetShape();
	b2PolygonShape* polyB = (b2PolygonShape*)fB->GetShape();

	//fill subject polygon from fixtureA polygon
	for (int i = 0; i < polyA->GetVertexCount(); i++)
		outputVertices.push_back(fA->GetBody()->GetWorldPoint(polyA->GetVertex(i)));

	//fill clip polygon from fixtureB polygon
	vector<b2Vec2> clipPolygon;
	for (int i = 0; i < polyB->GetVertexCount(); i++)
		clipPolygon.push_back(fB->GetBody()->GetWorldPoint(polyB->GetVertex(i)));

	b2Vec2 cp1 = clipPolygon[clipPolygon.size() - 1];
	for (int j = 0; j < clipPolygon.size(); j++) {
		b2Vec2 cp2 = clipPolygon[j];
		if (outputVertices.empty()) {
			printf("hello");
			return false;
		}
		vector<b2Vec2> inputList = outputVertices;
		outputVertices.clear();
		b2Vec2 s = inputList[inputList.size() - 1]; //last on the input list
		for (int i = 0; i < inputList.size(); i++) {
			b2Vec2 e = inputList[i];
			if (inside(cp1, cp2, e)) {
				if (!inside(cp1, cp2, s)) {
					outputVertices.push_back(intersection(cp1, cp2, s, e));
				}
				outputVertices.push_back(e);
			}
			else if (inside(cp1, cp2, s)) {
				outputVertices.push_back(intersection(cp1, cp2, s, e));
			}
			s = e;
		}
		cp1 = cp2;
	}

	return !outputVertices.empty();
}

bool inside(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 p) {
	return (cp2.x - cp1.x) * (p.y - cp1.y) > (cp2.y - cp1.y) * (p.x - cp1.x);
}

b2Vec2 intersection(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 s, b2Vec2 e) {
	b2Vec2 dc(cp1.x - cp2.x, cp1.y - cp2.y);
	b2Vec2 dp(s.x - e.x, s.y - e.y);
	float n1 = cp1.x * cp2.y - cp1.y * cp2.x;
	float n2 = s.x * e.y - s.y * e.x;
	float n3 = 1.0 / (dc.x * dp.y - dc.y * dp.x);
	return b2Vec2((n1 * dp.x - n2 * dc.x) * n3, (n1 * dp.y - n2 * dc.y) * n3);
}

b2Vec2 ComputeCentroid(vector<b2Vec2> vs, float& area) {
	int count = (int)vs.size();
	b2Assert(count >= 3);

	b2Vec2 c;
	c.Set(0.0f, 0.0f);
	area = 0.0f;

	b2Vec2 pRef(0.0f, 0.0f);
	const float32 inv3 = 1.0f / 3.0f;

	for (int32 i = 0; i < count; i++) {
		b2Vec2 p1 = pRef;
		b2Vec2 p2 = vs[i];
		b2Vec2 p3 = i + 1 < count ? vs[i + 1] : vs[0];

		b2Vec2 e1 = p2 - p1;
		b2Vec2 e2 = p3 - p1;

		float32 D = b2Cross(e1, e2);
		float32 triangleArea = 0.5f * D;
		area += triangleArea;

		c += triangleArea * inv3 * (p1 + p2 + p3);
	}

	if (area > b2_epsilon)
		c *= 1.0f / area;
	else
		area = 0;

	return c;
}

void applybuoyancy(b2Fixture* box, b2Fixture* water, float area, b2Vec2 gravity, b2Vec2 centroid) {
	float displacedMass = water->GetDensity() * area;
	b2Vec2 buoyancyForce = displacedMass * -1 * gravity;
	box->GetBody()->ApplyForce(buoyancyForce, centroid, true);
}

void applydrag(b2Fixture* box, b2Fixture* water, float area, b2Vec2 centroid) {
	b2Vec2 velDir = box->GetBody()->GetLinearVelocityFromWorldPoint(centroid) - water->GetBody()->GetLinearVelocityFromWorldPoint(centroid);

	float vel = velDir.Normalize();

	float dragMag = water->GetDensity() * vel * vel / 2;
	b2Vec2 dragForce = dragMag * -velDir;
	box->GetBody()->ApplyForce(dragForce, centroid, true);

	float angularDrag = area * -water->GetBody()->GetAngularVelocity();
	box->GetBody()->ApplyTorque(angularDrag, true);

}
