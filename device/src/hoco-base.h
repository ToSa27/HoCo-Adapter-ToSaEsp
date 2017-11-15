class HoCo
{
private:
	static bool isInitialized;
	static bool isConnected;
	static void InitDevice(char *config);
public:
	~HoCo();
	static void Init();
	static void Start();
	static void Stop();
};
