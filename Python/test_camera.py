import cv2
import numpy as np

def capture_imx219_v4l2():
    """
    Capture IMX219 en utilisant v4l2src directement (sans nvarguscamerasrc)
    Format: RG10 (RAW10 Bayer)
    """
    
    # Pipeline V4L2 pour IMX219 en RAW10
    # Le capteur envoie du RG10 (Red-Green 10-bit)
    pipeline = (
        "v4l2src device=/dev/video0 io-mode=2 ! "
        "video/x-bayer, format=rggb, width=1920, height=1080, framerate=30/1 ! "
        "appsink"
    )
    
    print("Ouverture pipeline V4L2...")
    print(pipeline)
    
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        print("Échec ouverture pipeline")
        return None
    
    print("Pipeline ouvert, lecture frame...")
    ret, frame = cap.read()
    cap.release()
    
    if not ret or frame is None:
        print("Échec lecture frame")
        return None
    
    print(f"Frame capturée: shape={frame.shape}, dtype={frame.dtype}")
    print(f"Min={frame.min()}, Max={frame.max()}")
    
    return frame

def decode_and_convert(frame):
    """
    Décoder la frame RAW et convertir en RGB
    """
    # La frame est probablement en uint8 aplati, besoin de reshape
    h, w = 1080, 1920
    
    # Si la frame est 1D, c'est du RAW10 compacté
    if len(frame.shape) == 1 or frame.shape[0] == 1:
        print("Frame 1D détectée, reshape nécessaire...")
        # Essayer de reshape selon la taille
        total_pixels = frame.size
        print(f"Total pixels: {total_pixels}")
        
        # Pour RAW10: 5 bytes = 4 pixels
        # Donc pour 1920x1080 pixels, on attend: 1920*1080*10/8 = 2,592,000 bytes
        expected_raw10 = 1920 * 1080 * 10 // 8  # 2,592,000
        
        if total_pixels == expected_raw10:
            print("Format RAW10 détecté (2,592,000 bytes)")
            return decode_raw10(frame, 1920, 1080)
        else:
            # Essayer reshape simple 2D
            try:
                gray = frame.reshape((1080, 1920))
                print(f"Reshape simple OK: {gray.shape}")
                # Convertir en RGB via débayérisage
                rgb = cv2.cvtColor(gray, cv2.COLOR_BAYER_RG2BGR)
                return rgb
            except:
                print(f"Impossible de reshape en (1080, 1920)")
                return None
    else:
        # Déjà 2D ou 3D
        if len(frame.shape) == 3:
            gray = frame[:,:,0]
        else:
            gray = frame
        
        rgb = cv2.cvtColor(gray.astype(np.uint8), cv2.COLOR_BAYER_RG2BGR)
        return rgb

def decode_raw10(data, width, height):
    """
    Décoder le format RAW10 compacté (MIPI CSI-2 style)
    4 pixels = 5 bytes (40 bits)
    """
    # Alignement sur 4 pixels
    width_aligned = (width + 3) & ~3
    bytes_per_line = width_aligned * 10 // 8
    expected_size = bytes_per_line * height
    
    print(f"Decoding RAW10: aligned={width_aligned}, bytes_per_line={bytes_per_line}")
    
    if len(data) < expected_size:
        print(f"ERREUR: données insuffisantes {len(data)} < {expected_size}")
        return None
    
    # Reshape en lignes
    raw = data[:expected_size].reshape(height, bytes_per_line)
    
    # Décoder les 5 bytes en 4 pixels
    # [P0[9:2], P1[9:2], P2[9:2], P3[9:2], P0[1:0]P1[1:0]P2[1:0]P3[1:0]]
    p0 = raw[:, 0::5].astype(np.uint16) << 2
    p1 = raw[:, 1::5].astype(np.uint16) << 2
    p2 = raw[:, 2::5].astype(np.uint16) << 2
    p3 = raw[:, 3::5].astype(np.uint16) << 2
    packed = raw[:, 4::5]
    
    # Extraire LSB
    p0 |= (packed >> 0) & 0x03
    p1 |= (packed >> 2) & 0x03
    p2 |= (packed >> 4) & 0x03
    p3 |= (packed >> 6) & 0x03
    
    # Reconstruire l'image
    image16 = np.zeros((height, width_aligned), dtype=np.uint16)
    image16[:, 0::4] = p0
    image16[:, 1::4] = p1
    image16[:, 2::4] = p2
    image16[:, 3::4] = p3
    
    # Tronquer et convertir en 8-bit
    image16 = image16[:, :width]
    image8 = (image16 >> 2).astype(np.uint8)
    
    print(f"Decoded: {image8.shape}, range [{image8.min()}-{image8.max()}]")
    
    # Débayérisage RGGB
    rgb = cv2.cvtColor(image8, cv2.COLOR_BAYER_RG2BGR)
    return rgb

# === TEST ===
if __name__ == "__main__":
    # Test capture
    raw_frame = capture_imx219_v4l2()
    
    if raw_frame is not None:
        # Sauvegarder le brut pour analyse
        np.save('frame_raw.npy', raw_frame)
        print(f"Sauvegardé: frame_raw.npy")
        
        # Décoder et convertir
        rgb = decode_and_convert(raw_frame)
        
        if rgb is not None:
            cv2.imwrite('capture_rgb.jpg', rgb)
            print(f"Sauvegardé: capture_rgb.jpg, shape={rgb.shape}")
        else:
            print("Échec décodage")
    else:
        print("Échec capture")