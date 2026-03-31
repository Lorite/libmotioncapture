#include "libmotioncapture/optitrack.h"

#include <boost/asio.hpp>
#include <cstring>
#include <iostream>

using boost::asio::ip::udp;

// Source - https://stackoverflow.com/a/3312896
// Posted by Steph, modified by community. See post 'Timeline' for change history
// Retrieved 2026-02-21, License - CC BY-SA 4.0

#ifdef __GNUC__
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop))
#endif


namespace libmotioncapture {

  constexpr int MAX_PACKETSIZE = 65503; // max size of packet (actual packet size is dynamic)
  constexpr int MAX_NAMELENGTH = 256;

  constexpr int NAT_CONNECT           = 0;
  constexpr int NAT_SERVERINFO        = 1;
  constexpr int NAT_REQUEST_MODELDEF  = 4;
  constexpr int NAT_MODELDEF          = 5;

  /**
   * \brief Unpack number of bytes of data for a given data type. 
   * Useful if you want to skip this type of data. 
   * \param ptr - input data stream pointer
   * \param major - NatNet major version
   * \param minor - NatNet minor version
   * \return - pointer after decoded object
  */
  char* UnpackDataSize(char* ptr, int major, int minor, int& nBytes, bool skip = false )
  {
      nBytes = 0;

      // size of all data for this data type (in bytes);
      if (((major == 4) && (minor > 0)) || (major > 4))
      {
          memcpy(&nBytes, ptr, 4); ptr += 4;
          // printf("Byte Count: %d\n", nBytes);
          if (skip)
          {
              ptr += nBytes;
          }
      }
      return ptr;
  }

  class MotionCaptureOptitrackImpl{
  public:
    MotionCaptureOptitrackImpl()
      : version()
      , versionMajor(0)
      , versionMinor(0)
      , io_context()
      , socket(io_context)
      , sender_endpoint()
      , data(MAX_PACKETSIZE)
    {
    }
    // void getObjectByRigidbody(
    //   const RigidBody& rb,
    //   Object& result) const
    //   {
    //     std::stringstream sstr;
    //     sstr << rb.id();
    //     const std::string name_number = sstr.str();
    //     std::string name_cf = "cf";
    //     const std::string name = name_cf + name_number;

    //     auto const translation = rb.location();
    //     auto const quaternion = rb.orientation();

    //     if(rb.trackingValid()) {
    //         Eigen::Vector3f position(
    //           -translation.y,     
    //           translation.x,
    //           translation.z);

    //         Eigen::Quaternionf rotation(
    //           quaternion.qw,
    //           -quaternion.qy,
    //           quaternion.qx,
    //           quaternion.qz
    //           );

    //         result = Object(name, position, rotation);

    //     } else {
    //         result = Object(name);
    //     }
    //   } 

    void parseModelDef(const char* data, size_t data_size)
    {
      const char *ptr = data;
      const char *end = data + data_size;

      auto canRead = [&](size_t n) {
        return ptr + n <= end;
      };

      auto consumeCString = [&]() {
        const void *term = std::memchr(ptr, '\0', static_cast<size_t>(end - ptr));
        if (!term) {
          return false;
        }
        ptr = static_cast<const char *>(term) + 1;
        return true;
      };

      auto readCString = [&](std::string& value) {
        const void *term = std::memchr(ptr, '\0', static_cast<size_t>(end - ptr));
        if (!term) {
          return false;
        }
        const char *term_ptr = static_cast<const char *>(term);
        value.assign(ptr, term_ptr - ptr);
        ptr = term_ptr + 1;
        return true;
      };

      const int major = versionMajor;
      const int minor = versionMinor;
      const bool has_description_size = (major > 4) || (major == 4 && minor >= 1);

      auto parseRigidBodyDescription = [&](const std::string& fallback_name) {
        std::string parsed_name;
        if (major >= 2)
        {
          if (!readCString(parsed_name)) {
            return false;
          }
        }

        if (!canRead(4)) {
          return false;
        }
        int ID = 0;
        memcpy(&ID, ptr, 4); ptr += 4;

        auto& def = rigidBodyDefinitions[ID];
        if (!parsed_name.empty()) {
          def.name = parsed_name;
        } else if (!fallback_name.empty() && def.name.empty()) {
          def.name = fallback_name;
        }
        def.ID = ID;

        if (!canRead(16)) {
          return false;
        }
        memcpy(&def.parentID, ptr, 4); ptr += 4;
        memcpy(&def.xoffset, ptr, 4); ptr += 4;
        memcpy(&def.yoffset, ptr, 4); ptr += 4;
        memcpy(&def.zoffset, ptr, 4); ptr += 4;

        // NatNet 4.2+ adds local orientation offsets to rigid body descriptions.
        if ((major > 4) || (major == 4 && minor >= 2))
        {
          if (!canRead(16)) {
            return false;
          }
          ptr += 16;
        }

        if (major >= 3)
        {
          if (!canRead(4)) {
            return false;
          }
          int nMarkers = 0;
          memcpy(&nMarkers, ptr, 4); ptr += 4;
          if (nMarkers < 0) {
            return false;
          }

          const size_t marker_positions_bytes = static_cast<size_t>(nMarkers) * 3U * sizeof(float);
          if (!canRead(marker_positions_bytes)) {
            return false;
          }
          ptr += marker_positions_bytes;

          const size_t marker_ids_bytes = static_cast<size_t>(nMarkers) * sizeof(int);
          if (!canRead(marker_ids_bytes)) {
            return false;
          }
          ptr += marker_ids_bytes;

          if (major >= 4)
          {
            for (int marker_idx = 0; marker_idx < nMarkers; ++marker_idx) {
              if (!consumeCString()) {
                return false;
              }
            }
          }
        }

        if (def.name.empty()) {
          def.name = "rigid_body_" + std::to_string(ID);
        }

        return true;
      };

      auto parseMarkerDescription = [&]() {
        if (!consumeCString()) {
          return false;
        }

        // marker ID (4) + position xyz (12) + size (4) + params (2)
        if (!canRead(22)) {
          return false;
        }
        ptr += 22;
        return true;
      };

      if (!canRead(4)) {
        return;
      }

      int MessageID = 0;
      int nBytes = 0;
      memcpy(&MessageID, ptr, 2); ptr += 2;
      memcpy(&nBytes, ptr, 2); ptr += 2;
      (void)nBytes;

      if (MessageID != NAT_MODELDEF) {
        return;
      }

      if (!canRead(4)) {
        return;
      }
      int nDatasets = 0;
      memcpy(&nDatasets, ptr, 4); ptr += 4;

      for (int i = 0; i < nDatasets; ++i)
      {
        if (!canRead(4)) {
          return;
        }
        int type = 0;
        memcpy(&type, ptr, 4); ptr += 4;

        int description_size = 0;
        const char *description_end = nullptr;
        if (has_description_size)
        {
          if (!canRead(4)) {
            return;
          }
          memcpy(&description_size, ptr, 4); ptr += 4;

          if (description_size < 0) {
            return;
          }
          if (!canRead(static_cast<size_t>(description_size))) {
            return;
          }
          description_end = ptr + description_size;
        }

        if (type == 1) // rigid body
        {
          if (!parseRigidBodyDescription("")) {
            return;
          }
        }
        else if (type == 6) // asset (NatNet 4.1+)
        {
          std::string asset_name;
          if (!readCString(asset_name)) {
            return;
          }

          // asset type + asset id
          if (!canRead(8)) {
            return;
          }
          ptr += 8;

          if (!canRead(4)) {
            return;
          }
          int nRigidBodies = 0;
          memcpy(&nRigidBodies, ptr, 4); ptr += 4;
          if (nRigidBodies < 0) {
            return;
          }

          for (int rb_idx = 0; rb_idx < nRigidBodies; ++rb_idx)
          {
            if (!parseRigidBodyDescription(asset_name)) {
              return;
            }
          }

          if (!canRead(4)) {
            return;
          }
          int nMarkers = 0;
          memcpy(&nMarkers, ptr, 4); ptr += 4;
          if (nMarkers < 0) {
            return;
          }

          for (int marker_idx = 0; marker_idx < nMarkers; ++marker_idx)
          {
            if (!parseMarkerDescription()) {
              return;
            }
          }
        }
        else if (!has_description_size)
        {
          if (type == 0) // markerset
          {
            if (!consumeCString()) {
              return;
            }

            if (!canRead(4)) {
              return;
            }
            int nMarkers = 0;
            memcpy(&nMarkers, ptr, 4); ptr += 4;

            for (int marker_idx = 0; marker_idx < nMarkers; ++marker_idx)
            {
              if (!consumeCString()) {
                return;
              }
            }
          }
          else if (type == 2) // skeleton
          {
            if (!consumeCString()) {
              return;
            }

            if (!canRead(8)) {
              return;
            }
            ptr += 4; // skeleton id
            int nRigidBodies = 0;
            memcpy(&nRigidBodies, ptr, 4); ptr += 4;

            for (int rb_idx = 0; rb_idx < nRigidBodies; ++rb_idx)
            {
              if (major >= 2)
              {
                if (!consumeCString()) {
                  return;
                }
              }

              if (!canRead(20)) {
                return;
              }
              ptr += 20;
            }
          }
          else
          {
            // Without per-dataset byte sizes we cannot safely skip unknown types.
            return;
          }
        }

        if (has_description_size)
        {
          if (ptr > description_end) {
            return;
          }
          ptr = description_end;
        }
      }
    }

  public:
    // NatNetClient client;
    std::string version;
    int versionMajor;
    int versionMinor;
    uint64_t clockFrequency; // ticks/second for timestamps

    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket socket;
    boost::asio::ip::udp::endpoint sender_endpoint;
    std::vector<char> data;

    struct rigidBody {
      int ID;
      float x;
      float y;
      float z;
      float qx;
      float qy;
      float qz;
      float qw;
      float fError; // mean marker error
      bool bTrackingValid;
    };
    std::vector<rigidBody> rigidBodies;

    struct marker {
      float x;
      float y;
      float z;
    };
    std::vector<marker> markers;

    struct rigidBodyDefinition {
      std::string name;
      int ID;
      int parentID;
      float xoffset;
      float yoffset;
      float zoffset;
    };
    std::map<int, rigidBodyDefinition> rigidBodyDefinitions;
  };

  MotionCaptureOptitrack::MotionCaptureOptitrack(
    const std::string &hostname,
    const std::string& interface_ip,
    int port_command)
  {
    pImpl = new MotionCaptureOptitrackImpl;

    // Connect to command port to query version
    boost::asio::io_context io_context_cmd;
    udp::socket socket_cmd(io_context_cmd, udp::endpoint(udp::v4(), 0));
    udp::resolver resolver_cmd(io_context_cmd);
    udp::endpoint endpoint_cmd(boost::asio::ip::make_address(hostname), port_command);

    typedef struct
    {
      unsigned short iMessage;                // message ID (e.g. NAT_FRAMEOFDATA)
      unsigned short nDataBytes;              // Num bytes in payload
    } sRequest;

    typedef struct
    {
      unsigned short iMessage;
      unsigned short nDataBytes;
      char szName[MAX_NAMELENGTH];      // host app's name
      unsigned char Version[4];         // host app's version [major.minor.build.revision]
      unsigned char NatNetVersion[4];   // host app's NatNet version [major.minor.build.revision]
      uint8_t HighResClockFrequency[8];   // host's high resolution clock frequency (ticks per second)
      uint16_t DataPort;
      bool IsMulticast;
      uint8_t MulticastGroupAddress[4];
    } sResponse;

    sRequest connectCmd = {NAT_CONNECT, 0};
    socket_cmd.send_to(boost::asio::buffer(&connectCmd, sizeof(connectCmd)), endpoint_cmd);

    sResponse response;
    udp::endpoint sender_endpoint;
    size_t reply_length = socket_cmd.receive_from(
        boost::asio::buffer(&response, sizeof(response)), sender_endpoint);

    if (response.iMessage != NAT_SERVERINFO) {
      throw std::runtime_error("Could not query NatNet version!");
    }

    std::ostringstream stringStream;
    stringStream << (int)response.NatNetVersion[0] << "."
                 << (int)response.NatNetVersion[1] << "."
                 << (int)response.NatNetVersion[2] << "."
                 << (int)response.NatNetVersion[3];
    pImpl->version = stringStream.str();

    pImpl->versionMajor = response.NatNetVersion[0];
    pImpl->versionMinor = response.NatNetVersion[1];
    memcpy(&pImpl->clockFrequency, response.HighResClockFrequency, sizeof(uint64_t));

    uint16_t port_data = response.DataPort;

    // query model def
    sRequest modelDefCmd = {NAT_REQUEST_MODELDEF, 0};
    socket_cmd.send_to(boost::asio::buffer(&modelDefCmd, sizeof(modelDefCmd)), endpoint_cmd);
    std::vector<char> modelDef(MAX_PACKETSIZE);
    reply_length = socket_cmd.receive_from(
        boost::asio::buffer(modelDef.data(), modelDef.size()), sender_endpoint);
    modelDef.resize(reply_length);
    pImpl->parseModelDef(modelDef.data(), modelDef.size());

      // ----------------------------------------------------------------------
// [NEW] Send a NatNet "Client Connect" handshake for unicast mode
// ----------------------------------------------------------------------

if (!response.IsMulticast) {
  std::cout << "[NatNet] Sending Client Connect request to server..." << std::endl;

  // Command port is usually 1510
  const uint16_t port_cmd = 1510;

  // Build a NatNet "Connect" command (message ID 0x0002)
  PACK(struct NatNetCommand {
    uint16_t messageId;
    uint16_t packetSize;
  });

  NatNetCommand connectCmd;
  connectCmd.messageId = 0x0002;  // "Client Connect" message
  connectCmd.packetSize = 0;

  boost::asio::ip::udp::socket socket_cmd_tmp(io_context_cmd);
  socket_cmd_tmp.open(boost::asio::ip::udp::v4());

  boost::asio::ip::udp::endpoint server_endpoint(
      boost::asio::ip::make_address_v4(hostname), port_cmd);

  boost::system::error_code ec;
  socket_cmd_tmp.send_to(boost::asio::buffer(&connectCmd, sizeof(connectCmd)),
                         server_endpoint, 0, ec);

  if (ec) {
    std::cerr << "[NatNet] Failed to send Client Connect request: " << ec.message() << std::endl;
  } else {
    std::cout << "[NatNet] Sent Client Connect request to "
              << hostname << ":" << port_cmd << std::endl;
  }

  // socket_cmd_tmp.close();
}


   // connect to data port to receive mocap data
auto listen_address_boost = boost::asio::ip::address_v4::any();
if (!interface_ip.empty() && interface_ip != "0.0.0.0") {
  boost::system::error_code address_ec;
  auto parsed_address = boost::asio::ip::make_address_v4(interface_ip, address_ec);
  if (!address_ec) {
    listen_address_boost = parsed_address;
  } else {
    std::cerr << "Invalid interface_ip '" << interface_ip
              << "', falling back to 0.0.0.0: " << address_ec.message() << std::endl;
  }
}

pImpl->socket.open(boost::asio::ip::udp::v4());
pImpl->socket.set_option(boost::asio::ip::udp::socket::reuse_address(true));

if (response.IsMulticast) {
  std::stringstream sstr;
  sstr << (int)response.MulticastGroupAddress[0] << "."
       << (int)response.MulticastGroupAddress[1] << "."
       << (int)response.MulticastGroupAddress[2] << "."
       << (int)response.MulticastGroupAddress[3];
  std::string multicast_address = sstr.str();
  auto multicast_address_boost = boost::asio::ip::make_address_v4(multicast_address);

  // Bind to any address for multicast
  boost::asio::ip::udp::endpoint listen_endpoint(boost::asio::ip::address_v4::any(), port_data);
  pImpl->socket.bind(listen_endpoint);

  // Join the multicast group on a specific interface
  pImpl->socket.set_option(boost::asio::ip::multicast::join_group(multicast_address_boost, listen_address_boost));

  std::cout << "Joined multicast group " << multicast_address
            << " on interface " << listen_address_boost << std::endl;

} else {
  // UNICAST MODE
  std::ostringstream ustr;
  ustr << "Using unicast from server " << hostname << ":" << port_data;
  std::cout << ustr.str() << std::endl;

  boost::system::error_code ec;

  boost::asio::ip::udp::endpoint local_endpoint(listen_address_boost, port_data);
  pImpl->socket.bind(local_endpoint, ec);

  if (ec) {
    std::cerr << "Failed to bind unicast UDP socket: " << ec.message() << std::endl;
  } else {
    std::cout << "Bound UDP socket to " << local_endpoint.address().to_string()
              << ":" << local_endpoint.port() << " for unicast reception." << std::endl;
  }
}
  }

  const std::string & MotionCaptureOptitrack::version() const
  {
    return pImpl->version;
  }

  void MotionCaptureOptitrack::waitForNextFrame()
  {
    // use a loop to get latest data
    do {
      pImpl->data.resize(MAX_PACKETSIZE);
      size_t length = pImpl->socket.receive_from(boost::asio::buffer(pImpl->data.data(), pImpl->data.size()), pImpl->sender_endpoint);
      pImpl->data.resize(length);
    } while (pImpl->socket.available() > 0);

    if (pImpl->data.size() > 4) {
      char *ptr = pImpl->data.data();
      int major = pImpl->versionMajor;
      int minor = pImpl->versionMinor;

      // First 2 Bytes is message ID
      int MessageID = 0;
      memcpy(&MessageID, ptr, 2); ptr += 2;
      // printf("Message ID : %d\n", MessageID);

      // Second 2 Bytes is the size of the packet
      int nBytes = 0;
      memcpy(&nBytes, ptr, 2); ptr += 2;
      // printf("Byte count : %d\n", nBytes);

      if(MessageID == 7)      // FRAME OF MOCAP DATA packet
      {
        // Next 4 Bytes is the frame number
        int frameNumber = 0; memcpy(&frameNumber, ptr, 4); ptr += 4;
        // printf("Frame # : %d\n", frameNumber);
      
        // Next 4 Bytes is the number of data sets (markersets, rigidbodies, etc)
        int nMarkerSets = 0; memcpy(&nMarkerSets, ptr, 4); ptr += 4;
        // printf("Marker Set Count : %d\n", nMarkerSets);

        int nBytes=0;
        ptr = UnpackDataSize(ptr, major, minor,nBytes);

        // Loop through number of marker sets and get name and data
        for (int i=0; i < nMarkerSets; i++)
        {
          ptr += strlen(ptr) + 1;
          int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
          ptr += nMarkers * 12;
        }

        // Loop through unlabeled markers
        // OtherMarker list is Deprecated
        int nOtherMarkers = 0; memcpy(&nOtherMarkers, ptr, 4); ptr += 4;
        ptr = UnpackDataSize(ptr, major, minor,nBytes);
        pImpl->markers.resize(nOtherMarkers);
        for (int j = 0; j < nOtherMarkers; j++)
        {
          memcpy(&pImpl->markers[j].x, ptr, 4); ptr += 4;
          memcpy(&pImpl->markers[j].y, ptr, 4); ptr += 4;
          memcpy(&pImpl->markers[j].z, ptr, 4); ptr += 4;
        }

        // Loop through rigidbodies
        int nRigidBodies = 0; memcpy(&nRigidBodies, ptr, 4); ptr += 4;
        ptr = UnpackDataSize(ptr, major, minor,nBytes);
        pImpl->rigidBodies.resize(nRigidBodies);
        // printf("Rigid Body Count : %d\n", nRigidBodies);
        for (int j=0; j < nRigidBodies; j++)
        {
          // Rigid body position and orientation 
          memcpy(&pImpl->rigidBodies[j].ID, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].x, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].y, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].z, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].qx, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].qy, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].qz, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].qw, ptr, 4); ptr += 4;

          // NatNet version 2.0 and later
          if(major >= 2)
          {
            // Mean marker error
            memcpy(&pImpl->rigidBodies[j].fError, ptr, 4); ptr += 4;
          }

          // NatNet version 2.6 and later
          if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) ) 
          {
            // params
            short params = 0; memcpy(&params, ptr, 2); ptr += 2;
            pImpl->rigidBodies[j].bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
          }
        } // Go to next rigid body

        // Skeletons (NatNet version 2.1 and later)
        // (we do not support skeletons)
        if( ((major == 2)&&(minor>0)) || (major>2))
        {
          int nSkeletons = 0; memcpy(&nSkeletons, ptr, 4); ptr += 4;
          // printf("Skeleton Count : %d\n", nSkeletons);
          ptr = UnpackDataSize(ptr, major, minor,nBytes);

          // Loop through skeletons
          for (int j=0; j < nSkeletons; j++)
          {
            // skeleton id
            // int skeletonID = 0;
            // memcpy(&skeletonID, ptr, 4);
            ptr += 4;

            // Number of rigid bodies (bones) in skeleton
            int nRigidBodies = 0;
            memcpy(&nRigidBodies, ptr, 4); ptr += 4;
            // printf("Rigid Body Count : %d\n", nRigidBodies);

            // Loop through rigid bodies (bones) in skeleton
            for (int j=0; j < nRigidBodies; j++)
            {
              // Rigid body position and orientation
              ptr += 8*4;

              // Mean marker error (NatNet version 2.0 and later)
              if(major >= 2)
              {
                ptr += 4;
              }

              // Tracking flags (NatNet version 2.6 and later)
              if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) ) 
              {
                ptr += 2;
              }
            } // next rigid body
          } // next skeleton
        }

        // Assets ( Motive 3.1 / NatNet 4.1 and greater)
        if (((major == 4) && (minor > 0)) || (major > 4))
        {
            int nAssets = 0;
            memcpy(&nAssets, ptr, 4); ptr += 4;
            // printf("Asset Count : %d\n", nAssets);

            int nBytes=0;
            ptr = UnpackDataSize(ptr, major, minor,nBytes);
            ptr += nBytes;
        }
        
        // labeled markers (NatNet version 2.3 and later)
        // labeled markers - this includes all markers: Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
        if( ((major == 2)&&(minor>=3)) || (major>2))
        {
          int nLabeledMarkers = 0;
          memcpy(&nLabeledMarkers, ptr, 4); ptr += 4;
          ptr = UnpackDataSize(ptr, major, minor,nBytes);
          pImpl->markers.resize(nOtherMarkers + nLabeledMarkers);
          // printf("Labeled Marker Count : %d\n", nLabeledMarkers);

          // Loop through labeled markers
          for (int j=0; j < nLabeledMarkers; j++)
          {
            // id
            // Marker ID Scheme:
            // Active Markers:
            //   ID = ActiveID, correlates to RB ActiveLabels list
            // Passive Markers: 
            //   If Asset with Legacy Labels
            //      AssetID   (Hi Word)
            //      MemberID  (Lo Word)
            //   Else
            //      PointCloud ID
            // int ID = 0; memcpy(&ID, ptr, 4);
            ptr += 4;
            // int modelID, markerID;
            // DecodeMarkerID(ID, &modelID, &markerID);

            memcpy(&pImpl->markers[nOtherMarkers + j].x, ptr, 4); ptr += 4;
            memcpy(&pImpl->markers[nOtherMarkers + j].y, ptr, 4); ptr += 4;
            memcpy(&pImpl->markers[nOtherMarkers + j].z, ptr, 4); ptr += 4;
            // size
            //float size = 0.0f; memcpy(&size, ptr, 4);
            ptr += 4;

            // NatNet version 2.6 and later
            if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) ) 
            {
              // marker params
              // short params = 0; memcpy(&params, ptr, 2);
              ptr += 2;
              // bool bOccluded = (params & 0x01) != 0;     // marker was not visible (occluded) in this frame
              // bool bPCSolved = (params & 0x02) != 0;     // position provided by point cloud solve
              // bool bModelSolved = (params & 0x04) != 0;  // position provided by model solve
              // if ((major >= 3) || (major == 0))
              // {
              //   bool bHasModel = (params & 0x08) != 0;     // marker has an associated asset in the data stream
              //   bool bUnlabeled = (params & 0x10) != 0;    // marker is 'unlabeled', but has a point cloud ID
              //   bool bActiveMarker = (params & 0x20) != 0; // marker is an actively labeled LED marker
              // }
            }

            // NatNet version 3.0 and later
            // float residual = 0.0f;
            if ((major >= 3) || (major == 0))
            {
              // Marker residual
              // memcpy(&residual, ptr, 4);
              ptr += 4;
            }
          }
        }

        // Force Plate data (NatNet version 2.9 and later)
        if (((major == 2) && (minor >= 9)) || (major > 2))
        {
          int nForcePlates;
          memcpy(&nForcePlates, ptr, 4); ptr += 4;
          ptr = UnpackDataSize(ptr, major, minor,nBytes);
          for (int iForcePlate = 0; iForcePlate < nForcePlates; iForcePlate++)
          {
            // ID
            // int ID = 0; memcpy(&ID, ptr, 4);
            ptr += 4;
            // printf("Force Plate : %d\n", ID);

            // Channel Count
            int nChannels = 0; memcpy(&nChannels, ptr, 4); ptr += 4;

            // Channel Data
            for (int i = 0; i < nChannels; i++)
            {
              // printf(" Channel %d : ", i);
              int nFrames = 0; memcpy(&nFrames, ptr, 4); ptr += 4;
              for (int j = 0; j < nFrames; j++)
              {
                  // float val = 0.0f;  memcpy(&val, ptr, 4);
                  ptr += 4;
                  // printf("%3.2f   ", val);
              }
              // printf("\n");
            }
          }
        }

        // Device data (NatNet version 3.0 and later)
        if (((major == 2) && (minor >= 11)) || (major > 2))
        {
          int nDevices;
          memcpy(&nDevices, ptr, 4); ptr += 4;
          ptr = UnpackDataSize(ptr, major, minor,nBytes);
          for (int iDevice = 0; iDevice < nDevices; iDevice++)
          {
            // ID
            // int ID = 0; memcpy(&ID, ptr, 4);
            ptr += 4;
            // printf("Device : %d\n", ID);

            // Channel Count
            int nChannels = 0; memcpy(&nChannels, ptr, 4); ptr += 4;

            // Channel Data
            for (int i = 0; i < nChannels; i++)
            {
              // printf(" Channel %d : ", i);
              int nFrames = 0; memcpy(&nFrames, ptr, 4); ptr += 4;
              for (int j = 0; j < nFrames; j++)
              {
                  // float val = 0.0f;  memcpy(&val, ptr, 4); 
                  ptr += 4;
                  // printf("%3.2f   ", val);
              }
              // printf("\n");
            }
          }
        }
    
        // software latency (removed in version 3.0)
        if ( major < 3 )
        {
          // float softwareLatency = 0.0f; memcpy(&softwareLatency, ptr, 4);
          ptr += 4;
          // printf("software latency : %3.3f\n", softwareLatency);
        }

        // timecode
        // unsigned int timecode = 0;  memcpy(&timecode, ptr, 4);
        ptr += 4;
        // unsigned int timecodeSub = 0; memcpy(&timecodeSub, ptr, 4);
        ptr += 4;
        // char szTimecode[128] = "";
        // TimecodeStringify(timecode, timecodeSub, szTimecode, 128);

        // timestamp
        // double timestamp = 0.0f;

        // NatNet version 2.7 and later - increased from single to double precision
        if( ((major == 2)&&(minor>=7)) || (major>2))
        {
          // memcpy(&timestamp, ptr, 8);
          ptr += 8;
        }
        else
        {
          // float fTemp = 0.0f;
          // memcpy(&fTemp, ptr, 4);
          ptr += 4;
          // timestamp = (double)fTemp;
        }
        // printf("Timestamp : %3.3f\n", timestamp);

        // high res timestamps (version 3.0 and later)
        latencies_.clear();
        if ( (major >= 3) || (major == 0) )
        {
          uint64_t cameraMidExposureTimestamp = 0;
          memcpy( &cameraMidExposureTimestamp, ptr, 8 );
          ptr += 8;

          uint64_t cameraDataReceivedTimestamp = 0;
          memcpy( &cameraDataReceivedTimestamp, ptr, 8 );
          ptr += 8;

          uint64_t transmitTimestamp = 0;
          memcpy( &transmitTimestamp, ptr, 8 );
          ptr += 8;

          const uint64_t cameraLatencyTicks = cameraDataReceivedTimestamp - cameraMidExposureTimestamp;
          const double cameraLatencySeconds = cameraLatencyTicks / (double)pImpl->clockFrequency;
          latencies_.emplace_back(LatencyInfo("Camera", cameraLatencySeconds));

          const uint64_t swLatencyTicks = transmitTimestamp - cameraDataReceivedTimestamp;
          const double swLatencySeconds = swLatencyTicks / (double)pImpl->clockFrequency;
          latencies_.emplace_back(LatencyInfo("Motive", swLatencySeconds));

          // convert actual shutter timestamp to microseconds
          timestamp_ = cameraMidExposureTimestamp * 1e6 / pImpl->clockFrequency;
        }

        // frame params
        short params = 0;  memcpy(&params, ptr, 2);
        ptr += 2;
        // bool bIsRecording = (params & 0x01) != 0;                  // 0x01 Motive is recording
        bool bTrackedModelsChanged = (params & 0x02) != 0;         // 0x02 Actively tracked model list has changed

        // end of data tag
        // int eod = 0; memcpy(&eod, ptr, 4); ptr += 4;
        // printf("End Packet\n-------------\n");
      }
      else
      {
          // Ignore packet types not handled by this parser.
      }
    }

  }

  const std::map<std::string, RigidBody>& MotionCaptureOptitrack::rigidBodies() const
  {
    // TODO: avoid copies here...
    rigidBodies_.clear();
    for (const auto& rb : pImpl->rigidBodies) {
      if (rb.bTrackingValid) {
        std::string name;
        float xoffset = 0.0f;
        float yoffset = 0.0f;
        float zoffset = 0.0f;

        const auto def_it = pImpl->rigidBodyDefinitions.find(rb.ID);
        if (def_it != pImpl->rigidBodyDefinitions.end()) {
          const auto& def = def_it->second;
          name = def.name;
          xoffset = def.xoffset;
          yoffset = def.yoffset;
          zoffset = def.zoffset;
        }

        if (name.empty()) {
          name = "rigid_body_" + std::to_string(rb.ID);
        }

        if (rigidBodies_.find(name) != rigidBodies_.end()) {
          name += "_" + std::to_string(rb.ID);
        }

        Eigen::Vector3f position(
          rb.x + xoffset,
          rb.y + yoffset,
          rb.z + zoffset);

        Eigen::Quaternionf rotation(
          rb.qw, // w
          rb.qx, // x
          rb.qy, // y
          rb.qz  // z
          );
        rigidBodies_.emplace(name, RigidBody(name, position, rotation));
      }
    }
    return rigidBodies_;
  }

  const PointCloud& MotionCaptureOptitrack::pointCloud() const
  {
    // TODO: avoid copies here...
    pointcloud_.resize(pImpl->markers.size(), Eigen::NoChange);
    for (size_t r = 0; r < pImpl->markers.size(); ++r) {
      const auto& marker = pImpl->markers[r];
      pointcloud_.row(r) << marker.x, marker.y, marker.z;
    }
    return pointcloud_;
  }

  const std::vector<LatencyInfo> &MotionCaptureOptitrack::latency() const
  {
    return latencies_;
  }

  uint64_t MotionCaptureOptitrack::timeStamp() const
  {
    return timestamp_;
  }

  MotionCaptureOptitrack::~MotionCaptureOptitrack()
  {
    delete pImpl;
  }

}
