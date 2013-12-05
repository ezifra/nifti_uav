#include "PJSSH.h"
#include <sstream>
#include <stdexcept>
#include <libssh2_sftp.h>
#include "libssh2.h"
#include "stdio.h"
#include <fcntl.h>
#include <errno.h>

using namespace std;

#if defined(linux) || defined(__linux) || defined(__linux__) || defined(__CYGWIN__)
#define PJSSH_POSIX
#include <netdb.h>  // gethostbyname
#elif defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
#define PJSSH_WINDOWS
#include <winsock2.h>
#endif

PJSSH::PJSSH(const char* aUserName, const char* aPassword,
		const char* aHostName, const int & aPortNumber) :
	LastErrorMessage(""), SessionIsOk(false), hSock(CreateSocketAndConnect(
			aHostName, aPortNumber)) {
	if (hSock > 0) {
		// Now, we have the socket, let's initialize the session.
		mSession = libssh2_session_init();
		if (libssh2_session_startup(mSession, hSock)) {
			LastErrorMessage = "Could not startup the ssh session.";
		} else {
			if (libssh2_userauth_password(mSession,aUserName,aPassword)) {
				LastErrorMessage = "Could not get authenticated.";
			} else {
				SessionIsOk = true;
			}
		}
	}
}

PJSSH::~PJSSH() {
	if (SessionIsOk) {
		libssh2_session_disconnect(mSession,"Goodbye from PJSSH.");
		libssh2_session_free(mSession);
	}
#ifdef PJSSH_WINDOWS
	closesocket(hSock);
	WSACleanup();
#else
#ifdef PJSSH_POSIX
	close(hSock);
#endif
#endif
}

void PJSSH::ExecuteCmd(const char* aCommand) const {
	if (!SessionIsOk) {
		std::ostringstream o;
		o << "Can not execute command since the SSH session is not set up ok. "
				<< "Last error message: " << LastErrorMessage;
		throw std::logic_error(o.str());
	}

	// Try to open a channel to be used for executing the command.
	LIBSSH2_CHANNEL* channel = libssh2_channel_open_session(mSession);
	if (NULL == channel) {
		throw std::runtime_error("Could not open communication channel for "
			"executing remote command.");
	}

	//  Execute the command.
	if (-1 == libssh2_channel_exec(channel,aCommand)) {
		throw std::runtime_error("Failed to execute the remote command.");
	}

	// Close the channel.
	libssh2_channel_close(channel);

	// Free resources.
	libssh2_channel_free(channel);
}

PJSSH::PJSSH(const PJSSH & source) {
}

PJSSH & PJSSH::operator=(const PJSSH & source) {
	return *this;
}

std::string PJSSH::readFile(std::string file) const {
	int rc;
	off_t got = 0;
	struct stat fileinfo;
	std::string result;

	LIBSSH2_CHANNEL *channel = libssh2_scp_recv(mSession, file.c_str(),
			&fileinfo);

	if (!channel) {
		fprintf(stderr, "Unable to open a session: %d\n",
				libssh2_session_last_errno(mSession));
	}

	while (got < fileinfo.st_size) {
		char mem[1024];
		int amount = sizeof(mem);

		if ((fileinfo.st_size - got) < amount) {
			amount = fileinfo.st_size - got;
		}

		rc = libssh2_channel_read(channel, mem, amount);

		if (rc > 0) {
			//write(1, mem, rc);
			result.append(mem);
		} else if (rc < 0) {
			fprintf(stderr, "libssh2_channel_read() failed: %d\n", rc);

			break;
		}
		got += rc;
	}

	libssh2_channel_free(channel);
	return result;
}

void PJSSH::GetFile(const char* aRemoteFileName, std::ostream& aStream) const {
	LIBSSH2_SFTP* sftp = libssh2_sftp_init(mSession);

	if (NULL == sftp) {
		throw std::runtime_error("Failed to open a sftp session.");
	}

	LIBSSH2_SFTP_HANDLE* file_handle =
			libssh2_sftp_open(sftp,aRemoteFileName,LIBSSH2_FXF_READ,0);

	if (NULL == file_handle) {
		std::ostringstream o;
		o << "Failed to open remote file for reading. Last error code="
				<< libssh2_sftp_last_error(sftp);
		throw std::runtime_error(o.str());
	}

	// Read the whole file and write the read data on the supplied stream.
	char buffer[1024];
	size_t num_of_read_bytes(0);
	do {
		num_of_read_bytes = libssh2_sftp_read(file_handle, buffer, 1024);
		aStream.write(buffer, num_of_read_bytes);
	} while (num_of_read_bytes == 1024);

	// Close sftp file handle and end SFTP session.
	libssh2_sftp_close_handle(file_handle);
	libssh2_sftp_shutdown(sftp);
}

void PJSSH::PutStream(std::istream & aStream, const char* aRemoteFileName) const {
	LIBSSH2_SFTP* sftp = libssh2_sftp_init(mSession);

	if (NULL == sftp) {
		throw std::runtime_error("Failed to open a sftp session.");
	}

	LIBSSH2_SFTP_HANDLE* file_handle = libssh2_sftp_open(sftp,aRemoteFileName,
			LIBSSH2_FXF_TRUNC | LIBSSH2_FXF_WRITE,0);

	if (NULL == file_handle) {
		std::ostringstream o;
		o << "Failed to write on remote file. Last error code="
				<< libssh2_sftp_last_error(sftp);
		throw std::runtime_error(o.str());
	}
	char buffer[1024];
	do {
		aStream.read(buffer, 1024);
		const std::streamsize num_of_read_characters(aStream.gcount());
		if (num_of_read_characters > 0) {
			const size_t num_of_bytes_written = libssh2_sftp_write(file_handle,
					buffer, num_of_read_characters);
			if (num_of_bytes_written == ((size_t) -1)) {
				throw std::runtime_error("Failed to write to the remote file.");
			} else if (static_cast<std::streamsize> (num_of_bytes_written)
					!= num_of_read_characters) {
				throw std::runtime_error(
						"Failed to write all bytes to remote file.");
			} else {
				throw std::runtime_error(
						"Failed to read characters from the input "
							"stream to be written to the remote file.");
			}
		}
	} while (aStream);

	// Close sftp file handle and end SFTP session.
	libssh2_sftp_close_handle(file_handle);
	libssh2_sftp_shutdown(sftp);
}

int PJSSH::CreateSocketAndConnect(const char* aHostName, const int& aPortNumber) {
#ifdef PJSSH_WINDOWS
	int hSock(INVALID_SOCKET);
	WSADATA wsaData = {0};
	if( 0 != WSAStartup( WINSOCK_VERSION, &wsaData ))
	{
		LastErrorMessage = "Failed to initialize communication.";
	}
	else
	{
		hSock = socket( AF_INET, SOCK_STREAM, IPPROTO_IP );
		// Set the address to use for communication.
		sockaddr_in saServer = {0};
		saServer.sin_family = PF_INET;
		saServer.sin_port = htons( aPortNumber );
		saServer.sin_addr.s_addr = inet_addr( aHostName );

		if( SOCKET_ERROR == connect(hSock,(sockaddr*)&saServer,sizeof(sockaddr)))
		{
			LastErrorMessage = "Failed to connect.";
			hSock = INVALID_SOCKET;
		}
		else
		{
			if( hSock == INVALID_SOCKET )
			{
				LastErrorMessage = "Unable to connect.";
			}
		}
	}
	return hSock;
#else
#ifdef PJSSH_POSIX
	int hSock(-1);
	struct hostent* hp = gethostbyname(aHostName);
	if (!hp) {
		LastErrorMessage = "Failed to get IP adress of server.";
	} else {
		struct sockaddr_in s;
		s.sin_addr = *(struct in_addr *) hp->h_addr_list[0];
		s.sin_family = hp->h_addrtype;
		s.sin_port = htons(aPortNumber);
		hSock = socket(AF_INET, SOCK_STREAM, 0);
		if (hSock < 0) {
			LastErrorMessage = "Failed to create socket.";
		} else {
			if (connect(hSock, (struct sockaddr *) &s, sizeof(s)) < 0) {
				LastErrorMessage = "Failed to connect.";
				hSock = -1;
			}
		}
	}
	return hSock;
#else
#error "Need to get a socket for this platform!"
#endif
#endif
}
