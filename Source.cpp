#define _USE_MATH_DEFINES
#define ENDIAN(x) (((x) >> 8) | ((((x)<<8) & 0xff00)))
#define WRITEWORD(x) file.write((char*)&(x),2);
#include<iostream>
#include<Windows.h>
#include<fstream>
#include<vector>
#include<string>
#include<cmath>
#include<algorithm>
#include<unordered_map>
#include<array>


struct Color
{
	BYTE b;
	BYTE g;
	BYTE r;
};
class BMP
{
public:
	BMP(const std::string& p);
private:
	std::vector<Color> color;
	int width = 0;
	int height = 0;
	int padding = 0;
	int padded_width = 0;
	friend class BMPTOJPG;
};

class BMPTOJPG
{
public:
	BMPTOJPG(BMP* bmp, const std::string& path);
public:
	struct MCU
	{
		int data[3][64] = {0};
	};
private:
	struct BitWriter
	{
		std::vector<BYTE> data = {0};
		void WriteBit(BYTE b)
		{
			data[data.size() - 1] |= b << pos;
			if (pos == 0)
			{
				pos = 7;
				data.push_back(0);
			}
			else
			{
				pos--;
			}

		}

		void WriteBits(int w, int length)
		{
			for (; length > 0; length--)
			{
				BYTE bit = (w >> (length - 1)) & 1;
				WriteBit(bit);
			}
		}

		void WriteIngoringLeadingZeroes(int w)
		{
			int start = log2(w);
			for (; start >= 0; start--)
			{
				WriteBit((w >> start) & 1);
			}
		};
	private:
		BYTE pos = 7;
	};
	struct Symbol
	{
		Symbol() :symbol(0), coeff(0) {};
		Symbol(BYTE s, int c, int l) :symbol(s), coeff(c), length(l) {};
		BYTE symbol = 0;
		int coeff = 0;
		int length = 0;
	};

	struct HuffmanCode
	{
		UINT code;
		UINT length;
	};
	struct node
	{
		BYTE symbol = 0xff;
		int freq = 1;
		node* parent = nullptr;
		node* left = nullptr;
		node* right = nullptr;
	};

private:
	void RGBToYCbCr(MCU& mcu);
	void DCT(int* component);
	void Quantization(int* component, BYTE table[64]);
	void EncodeMCU(int* component, Symbol& DCSymbol, std::vector<Symbol>& ACSymbol, int& prev_DC);//codes symbols
	std::unordered_map<BYTE, HuffmanCode> GenerateHuffmanDCCodes();// symbol -> code
	std::unordered_map<BYTE, HuffmanCode> GenerateHuffmanACCodes();// symbol -> code
	std::unordered_map<BYTE, HuffmanCode> HuffmanEncoding(std::unordered_map<BYTE, node*>& map, std::vector<BYTE>* hTable);
	void HuffmanEncodingDFS(node* n, std::vector<BYTE>* hTable, int depth);

	template<class T>
	constexpr void Zigzag(T arr[64]);
private:
	BYTE zigZagMap[64] ={
	 0,  1,  8, 16,  9,  2,  3, 10,
	17, 24, 32, 25, 18, 11,  4,  5,
	12, 19, 26, 33, 40, 48, 41, 34,
	27, 20, 13,  6,  7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36,
	29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46,
	53, 60, 61, 54, 47 ,55 ,62, 63 };
	BYTE quantizationTableLuminace[64] = {
	16,11,10,16,24,40,51,64,
	12,12,14,19,26,58,60,55,
	14,13,16,24,40,57,69,56,
	14,17,22,29,51,87,80,62,
	18,22,37,56,68,109,103,77,
	24,36,55,64,81,104,113,92,
	49,64,78,87,103,121,120,101,
	72,92,95,98,112,100,103,99
	};
	BYTE quantizationTableChrominace[64] = {
	17,18,24,47,99,99,99,99,
	18,21,26,66,99,99,99,99,
	24,26,56,99,99,99,99,99,
	47,66,99,99,99,99,99,99,
	99,99,99,99,99,99,99,99,
	99,99,99,99,99,99,99,99,
	99,99,99,99,99,99,99,99,
	99,99,99,99,99,99,99,99
	};
	std::vector<Symbol> DCsymbols;
	std::vector<std::array<std::vector<Symbol>,3>> ACsymbols;
	std::vector<BYTE> coeff;
	std::vector<BYTE> HuffmanDCTable[16];
	std::vector<BYTE> HuffmanACTable[16];
};
const BYTE SOF0 = 0xC0;
const BYTE DHT = 0xC4;
const BYTE SOI = 0xD8;
const BYTE EOI = 0xD9;
const BYTE SOS = 0xDA;
const BYTE DQT = 0xDB;
const BYTE APP0 = 0xE0;



void writebmp(BMPTOJPG::MCU* mcus, int width, int height, int mcuwidth)
{
	std::ofstream file("bbb.bmp", std::ios::binary);
	if (!file.is_open())
	{
		std::cout << "ERROR - Cannot Open the output file\n";
		return;
	}
	const UINT padding = width % 4;
	BITMAPFILEHEADER bfh;
	bfh.bfType = 0x4d42;
	bfh.bfSize = height * width * 3 + sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);
	bfh.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);
	bfh.bfReserved1 = 0;
	bfh.bfReserved2 = 0;
	BITMAPINFOHEADER bih;
	bih.biSize = sizeof(BITMAPINFOHEADER);
	bih.biBitCount = 24;
	bih.biPlanes = 1;
	bih.biCompression = BI_RGB;
	bih.biHeight = height;
	bih.biWidth = width;
	bih.biClrImportant = 0;
	bih.biClrUsed = 0;
	bih.biSizeImage = height * width * 3;
	bih.biXPelsPerMeter = 0;
	bih.biYPelsPerMeter = 0;
	try
	{
		file.write(reinterpret_cast<char*>(&bfh), sizeof(BITMAPFILEHEADER));
		file.write(reinterpret_cast<char*>(&bih), sizeof(BITMAPINFOHEADER));
	}
	catch (std::exception& e)
	{
		int trap = 0;
	}
	for (UINT y = height - 1; y < height; y--)
	{
		const UINT mcuRow = y / 8;
		const UINT pixelRow = y % 8;
		for (UINT x = 0; x < width; x++)
		{
			const UINT mcuColumn = x / 8;
			const UINT pixelColumn = x % 8;
			const UINT mcuIndex = mcuRow * mcuwidth + mcuColumn;
			const UINT pixelIndex = pixelRow * 8 + pixelColumn;
			file.put(mcus[mcuIndex].data[2][pixelIndex]);
			file.put(mcus[mcuIndex].data[1][pixelIndex]);
			file.put(mcus[mcuIndex].data[0][pixelIndex]);
		}
		for (UINT i = 0; i < padding; i++)
		{
			file.put(0);
		}
	}
	file.close();
}



int main()
{
	BMP bmp("test.bmp");
	BMPTOJPG jpg(&bmp, "test.jpg");
	return 0;
}

BMP::BMP(const std::string& p)
{
	std::ifstream file(p, std::ios::binary);
	BITMAPFILEHEADER bfh;
	BITMAPINFOHEADER bih;
	if (file.is_open())
	{
		file.read((char*)&bfh, sizeof(BITMAPFILEHEADER));
		file.read((char*)&bih, sizeof(BITMAPINFOHEADER));
		width = bih.biWidth;
		height = bih.biHeight;
		padding = (4 - (width % 4)) % 4;
		padded_width = width + padding;
		color.resize(height * padded_width * sizeof(Color));
		file.read((char*)color.data(), color.size());
		file.close();
	}
}

BMPTOJPG::BMPTOJPG(BMP* bmp, const std::string& path)
{
	std::ofstream file(path, std::ios::binary);
	if (!file.is_open())
	{
		std::cout << "GG\n";
		return;
	}

	int mcuWidth = (bmp->width + 7) / 8;
	int mcuHeight = (bmp->height + 7) / 8;
	MCU* mcus = new MCU[mcuWidth * mcuHeight];
	ACsymbols.resize(mcuWidth * mcuHeight);
	DCsymbols.resize(mcuWidth * mcuHeight * 3);
	//fill the mcus
	for (int i = 0; i < bmp->height; i++)
	{
		int mcu_y = i / 8;
		for (int j = 0; j < bmp->width; j++)
		{
			int bmpIndex = (bmp->height - i -1) * bmp->padded_width + j;
			int mcu_x = j / 8;
			int pixel_x = j % 8;
			int pixel_y = i % 8;
			int pixel = pixel_y * 8 + pixel_x;

			mcus[mcu_y * mcuWidth + mcu_x].data[0][pixel] = bmp->color[bmpIndex].r;
			mcus[mcu_y * mcuWidth + mcu_x].data[1][pixel] = bmp->color[bmpIndex].g;
			mcus[mcu_y * mcuWidth + mcu_x].data[2][pixel] = bmp->color[bmpIndex].b;
		}
	}

	//convert RGB to YCbCr
	for (int i = 0; i < mcuHeight * mcuWidth; i++)
	{
		RGBToYCbCr(mcus[i]);
	}

	


	//DCT & Zigzag
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < mcuHeight * mcuWidth; j++)
		{
			DCT(mcus[j].data[i]);
			Zigzag(mcus[j].data[i]);
		}
	}
	Zigzag(quantizationTableChrominace);
	Zigzag(quantizationTableLuminace);
	//Quantization
	for (int i = 0; i < mcuHeight * mcuWidth; i++)
	{
		Quantization(mcus[i].data[0], quantizationTableLuminace);
		Quantization(mcus[i].data[1], quantizationTableChrominace);
		Quantization(mcus[i].data[2], quantizationTableChrominace);
	}
	




	//Run-Length and Huffman encoding

	int prev_DCs[3] = { 0 };
	for (int i = 0; i < mcuHeight * mcuWidth; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			EncodeMCU(mcus[i].data[j], DCsymbols[i * 3 + j], ACsymbols[i][j], prev_DCs[j]);
		}
	}
	std::unordered_map<BYTE, HuffmanCode> DCmap = GenerateHuffmanDCCodes();
	std::unordered_map<BYTE, HuffmanCode> ACmap = GenerateHuffmanACCodes();
	//construct jpg
	struct
	{
		BYTE first=0xff;
		BYTE last = SOI;

	}SOI_T;


	struct
	{
		BYTE first = 0xff;
		BYTE last = DQT;
		WORD size = ENDIAN(67);
		BYTE tableInfo;
	}DQT_T;


	struct
	{
		BYTE first = 0xff;
		BYTE last = DHT;
		WORD size;
		BYTE info;
	}DHT_T;

	struct
	{
		BYTE first = 0xff;
		BYTE last = SOF0;
		WORD size = ENDIAN(17);
		BYTE precision = 8;
		WORD height;
		WORD width;
		BYTE numComponents = 3;
		struct
		{
			BYTE componentID;
			BYTE samplingFactor;
			BYTE quatizationTableID;
		}components[3];

	}SOF_T;


	struct
	{
		BYTE first = 0xff;
		BYTE last = SOS;
		WORD size = ENDIAN(12);
		BYTE numComponents = 3;
		struct
		{
			BYTE ID;
			BYTE HuffmanTableID;
		}component[3];
		BYTE startOfSelection = 0;
		BYTE endOfSelection = 63;
		BYTE successiveApproximation = 0;
	}SOS_T;
	
	//SOI
	file.write((char*)&SOI_T, 2);
	
	//APP0
	file.put(0xff); file.put(APP0);
	file.put(0x00); file.put(0x10); file.put(0x4a); file.put(0x46); file.put(0x49); file.put(0x46); file.put(0x00); file.put(0x01);
	file.put(0x01); file.put(0x01); file.put(0x00); file.put(0x60); file.put(0x00); file.put(0x60); file.put(0x00); file.put(0x00);
	//DQT
	DQT_T.tableInfo = 0;
	file.write((char*)&DQT_T, 5);
	file.write((char*)quantizationTableLuminace, sizeof(quantizationTableLuminace));
	DQT_T.tableInfo = 1;
	file.write((char*)&DQT_T, 5);
	file.write((char*)quantizationTableChrominace, sizeof(quantizationTableChrominace));
	//SOF
	SOF_T.width = ENDIAN(bmp->width);
	SOF_T.height = ENDIAN(bmp->height);
	SOF_T.components[0] = { 1,0x11,0 };
	SOF_T.components[1] = { 2,0x11,1 };
	SOF_T.components[2] = { 3,0x11,1 };
	file.put(SOF_T.first); file.put(SOF_T.last); WRITEWORD(SOF_T.size) file.put(SOF_T.precision); WRITEWORD(SOF_T.height)WRITEWORD(SOF_T.width);
	file.put(SOF_T.numComponents); for (int i = 0; i < 3; i++) { file.put(SOF_T.components[i].componentID); file.put(SOF_T.components[i].samplingFactor); file.put(SOF_T.components[i].quatizationTableID);}
	//DHT
	//DC
	DHT_T.info = 0x00;
	int size = 0;
	for (int i = 0; i < 16; i++)
	{
		size += HuffmanDCTable[i].size();
	}
	DHT_T.size = ENDIAN(19 + size);
	file.write((char*)&DHT_T, 5);
	for (int i = 0; i < 16; i++)
	{
		file.put(HuffmanDCTable[i].size());
	}
	for (int i = 0; i < 16; i++)
	{
		for (int j = 0; j < HuffmanDCTable[i].size(); j++)
		{
			file.put(HuffmanDCTable[i][j]);
		}
	}
	//AC
	DHT_T.info = 0x10;
	size = 0;
	for (int i = 0; i < 16; i++)
	{
		size += HuffmanACTable[i].size();
	}
	DHT_T.size = ENDIAN(19 + size);
	file.write((char*)&DHT_T, 5);
	for (int i = 0; i < 16; i++)
	{
		file.put(HuffmanACTable[i].size());
	}
	for (int i = 0; i < 16; i++)
	{
		for (int j = 0; j < HuffmanACTable[i].size(); j++)
		{
			file.put(HuffmanACTable[i][j]);
		}
	}
	//SOS
	SOS_T.component[0].ID = 1;
	SOS_T.component[0].HuffmanTableID = 0x00;

	SOS_T.component[1].ID = 2;
	SOS_T.component[1].HuffmanTableID = 0x00;

	SOS_T.component[2].ID = 3;
	SOS_T.component[2].HuffmanTableID = 0x00;
	file.write((char*)&SOS_T, 14);
	//bit stream
	BitWriter b;
	for (int i = 0; i < mcuHeight * mcuWidth; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			int DCIndex = i * 3 + j;
			Symbol& DCSymbol = DCsymbols[DCIndex];
			HuffmanCode code = DCmap[DCSymbol.symbol];
			b.WriteBits(code.code, code.length);
			b.WriteBits(DCSymbol.coeff, DCSymbol.length);

			for (int k = 0; k < ACsymbols[i][j].size(); k++)
			{
				Symbol& ACSymbol = ACsymbols[i][j][k];
				code = ACmap[ACSymbol.symbol];
				b.WriteBits(code.code,code.length);
				b.WriteBits(ACSymbol.coeff, ACSymbol.length);
			}
		}
	}
	for (int i = 0; i < b.data.size(); i++)
	{
		file.put(b.data[i]);
		if (b.data[i] == 0xff)
		{
			file.put(0x00);
		}
	}
	file.put(0xff);
	file.put(EOI);

	delete[] mcus;
	file.close();
}

void BMPTOJPG::RGBToYCbCr(MCU& mcu)
{
	for (int i = 0; i < 64; i++)
	{
		int Y = mcu.data[0][i] * 0.301 + mcu.data[1][i] * 0.586 + mcu.data[2][i] * 0.113;
		int Cb = mcu.data[0][i] * -0.168 + mcu.data[1][i] * -0.332 + mcu.data[2][i] * 0.500;
		int Cr = mcu.data[0][i] * 0.500 + mcu.data[1][i] * -0.417 + mcu.data[2][i] * -0.082;

		mcu.data[0][i] = Y - 128;
		mcu.data[1][i] = Cb;
		mcu.data[2][i] = Cr;
	}
}


inline float c(int u)
{
	return u == 0 ? M_SQRT1_2 : 1.0;
};
void BMPTOJPG::DCT(int* component)
{
	int result[64];
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 8; j++)
		{
			float sum = 0;
			for (int x = 0; x < 8; x++)
			{
				for (int y = 0; y < 8; y++)
				{
					sum += component[x * 8 + y] * cos((2 * x + 1) * i * M_PI / 16) * cos((2 * y + 1) * j * M_PI / 16);
				}
			}
			result[i * 8 + j] = 1.0 / 4.0 * c(i) * c(j) * sum;
		}
	}
	for (int i = 0; i < 64; i++)
	{
		component[i] = result[i];
	}

}

void BMPTOJPG::Quantization(int* component, BYTE table[64])
{
	for (int i = 0; i < 64; i++)
	{
		component[i] /= table[i];
	}
}

void BMPTOJPG::EncodeMCU(int* component, Symbol& DCSymbol, std::vector<Symbol>& ACSymbol, int& prev_DC)
{
	//DC
	int coeff = component[0] - prev_DC;
	
	if (coeff == 0)
	{
		DCSymbol = { 0, 0 ,0 };
	}
	else
	{
		int length = log2(abs(coeff))+1;
		if (coeff < 0)
		{
			coeff += (1 << length) - 1;
		}
		DCSymbol = { (BYTE)length, coeff, length };//{ symbol, coefficient }
	}
	prev_DC = component[0];
	//AC
	
	for (int i = 1; i < 64; i++)
	{
		int zeroes = 0;
		for (; i < 64; i++, zeroes++)
		{
			if (component[i] != 0)
			{
				break;
			}
		}
		if (i == 64)
		{
			ACSymbol.emplace_back(0x00, 0 ,0);
			break;
		}
		int length = log2(abs(component[i])) + 1;
		coeff = component[i];
		if (coeff < 0)
		{
			coeff += (1 << length) - 1;
		}
		while (zeroes > 15)
		{
			ACSymbol.emplace_back( 0xf0, 0 ,0);
			zeroes -= 16;
		}
		int symbol = (zeroes << 4) | length;
		ACSymbol.emplace_back((BYTE)symbol, coeff, length);
	}
}

std::unordered_map<BYTE, BMPTOJPG::HuffmanCode> BMPTOJPG::GenerateHuffmanDCCodes()
{
	std::unordered_map<BYTE, node*> map;
	for (auto i : DCsymbols)
	{
		if (map.find(i.symbol) == map.end())
		{
			map[i.symbol] = new node{ i.symbol, 1,nullptr, nullptr, nullptr };
		}
		else
		{
			map[i.symbol]->freq++;
		}
	}
	return HuffmanEncoding(map, HuffmanDCTable);
}

std::unordered_map<BYTE, BMPTOJPG::HuffmanCode> BMPTOJPG::GenerateHuffmanACCodes()
{
	std::unordered_map<BYTE, node*> map;
	for (int i = 0; i < ACsymbols.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < ACsymbols[i][j].size(); k++)
			{
				if (map.find(ACsymbols[i][j][k].symbol) == map.end())
				{
					map[ACsymbols[i][j][k].symbol] = new node{ ACsymbols[i][j][k].symbol, 1,nullptr, nullptr, nullptr };
				}
				else
				{
					map[ACsymbols[i][j][k].symbol]->freq++;
				}
			}
		}
	}
	return HuffmanEncoding(map, HuffmanACTable);
}

std::unordered_map<BYTE, BMPTOJPG::HuffmanCode> BMPTOJPG::HuffmanEncoding(std::unordered_map<BYTE, node*>& map, std::vector<BYTE>* hTable)
{
	std::vector<node*> vec(map.size());
	int i = 0;
	for (auto a : map)
	{
		vec[i] = a.second;
		i++;
	}
	sort(vec.begin(), vec.end(), [](node* a, node* b) {

		if (a == nullptr || b == nullptr)
		{
			return false;
		}
		return a->freq < b->freq;
	});
	while (vec.size()!=1)
	{
		node* combine = new node;
		combine->freq = vec[0]->freq + vec[1]->freq;
		combine->left = vec[0];
		combine->right = vec[1];
		vec[0]->parent = combine;
		vec[1]->parent = combine;
		vec[0] = combine;
		vec.erase(vec.begin()+1);
		sort(vec.begin(), vec.end(), [](node* a, node* b) {

			if (a == nullptr || b == nullptr)
			{
				return false;
			}
			return a->freq < b->freq;
		});
	}
	std::unordered_map<BYTE, HuffmanCode> m;
	HuffmanEncodingDFS(vec[0], hTable, 0);

	UINT code = 0;
	for (UINT i = 0; i < 16; i++)
	{
		for (UINT j = 0; j < hTable[i].size(); j++)
		{
			m[hTable[i][j]] = HuffmanCode{ code , i + 1 };
			code++;
		}
		code <<= 1;
	}

	return m;
}

void BMPTOJPG::HuffmanEncodingDFS(node* n,  std::vector<BYTE>* hTable, int depth)
{
	if (n->right == nullptr && n->left == nullptr)
	{
		if (n->symbol != 0xff)
		{
			if (depth != 0)
			{
				depth--;
			}
			hTable[depth].push_back(n->symbol);
		}
		delete n;
		return;
	}
	if (n->right != nullptr)
	{
		HuffmanEncodingDFS(n->right, hTable, depth + 1);
	}
	if (n->left != nullptr)
	{
		HuffmanEncodingDFS(n->left, hTable, depth + 1);
	}
}

template<class T>
constexpr void BMPTOJPG::Zigzag(T arr[64])
{
	T tem[64];
	for (int i = 0; i < 64; i++)
	{
		tem[i] = arr[zigZagMap[i]];
	}
	for (int i = 0; i < 64; i++)
	{
		arr[i] = tem[i];
	}
}
