#!/bin/sh
set -eu

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo "Usage: $0 CAPTURE.ts [REPORT_PREFIX]" >&2
    exit 2
fi

INPUT=$1
PREFIX=${2:-${INPUT%.*}.network_validation}
MAX_AV_DURATION_DELTA_MS=${MAX_AV_DURATION_DELTA_MS:-300}

command -v ffprobe >/dev/null 2>&1 || { echo "ffprobe not found" >&2; exit 1; }
command -v ffmpeg >/dev/null 2>&1 || { echo "ffmpeg not found" >&2; exit 1; }
[ -s "$INPUT" ] || { echo "capture missing or empty: $INPUT" >&2; exit 1; }

STREAMS="${PREFIX}.streams.txt"
VIDEO_PACKETS="${PREFIX}.video_packets.txt"
AUDIO_PACKETS="${PREFIX}.audio_packets.txt"
DECODE_LOG="${PREFIX}.decode.log"
SUMMARY="${PREFIX}.summary.txt"

ffprobe -v error -show_format -show_streams "$INPUT" > "$STREAMS"

vcodec=$(ffprobe -v error -select_streams v:0 \
    -show_entries stream=codec_name -of default=nw=1:nk=1 "$INPUT")
acodec=$(ffprobe -v error -select_streams a:0 \
    -show_entries stream=codec_name -of default=nw=1:nk=1 "$INPUT")
asample_rate=$(ffprobe -v error -select_streams a:0 \
    -show_entries stream=sample_rate -of default=nw=1:nk=1 "$INPUT")
achannels=$(ffprobe -v error -select_streams a:0 \
    -show_entries stream=channels -of default=nw=1:nk=1 "$INPUT")
vduration=$(ffprobe -v error -select_streams v:0 \
    -show_entries stream=duration -of default=nw=1:nk=1 "$INPUT")
aduration=$(ffprobe -v error -select_streams a:0 \
    -show_entries stream=duration -of default=nw=1:nk=1 "$INPUT")
vtime_base=$(ffprobe -v error -select_streams v:0 \
    -show_entries stream=time_base -of default=nw=1:nk=1 "$INPUT")
atime_base=$(ffprobe -v error -select_streams a:0 \
    -show_entries stream=time_base -of default=nw=1:nk=1 "$INPUT")

[ "$vcodec" = "hevc" ] || { echo "expected HEVC, got: $vcodec" >&2; exit 1; }
[ "$acodec" = "aac" ] || { echo "expected AAC, got: $acodec" >&2; exit 1; }
[ "$asample_rate" = "48000" ] || {
    echo "expected 48000 Hz audio, got: $asample_rate" >&2
    exit 1
}
[ "$achannels" = "2" ] || {
    echo "expected stereo audio, got channels=$achannels" >&2
    exit 1
}

ffprobe -v error -select_streams v:0 -show_packets \
    -show_entries packet=pts_time,dts_time,duration_time,size,flags \
    -of compact=p=0:nk=0 "$INPUT" > "$VIDEO_PACKETS"
ffprobe -v error -select_streams a:0 -show_packets \
    -show_entries packet=pts_time,dts_time,duration_time,size,flags \
    -of compact=p=0:nk=0 "$INPUT" > "$AUDIO_PACKETS"

check_packets() {
    file=$1
    name=$2
    awk -F'|' -v name="$name" '
        function value(key,   i,a) {
            for (i=1; i<=NF; ++i) {
                split($i,a,"=")
                if (a[1] == key) return a[2]
            }
            return ""
        }
        BEGIN { n=0; have_pts=0; have_dts=0; bad=0; eps=0.000001 }
        {
            pts=value("pts_time"); dts=value("dts_time"); dur=value("duration_time")
            if (pts != "" && pts != "N/A") {
                x=pts+0.0
                if (have_pts && x+eps < last_pts) {
                    printf "%s PTS regression packet %d: %.9f < %.9f\n", name,n,x,last_pts > "/dev/stderr"
                    bad=1
                }
                last_pts=x; have_pts=1
            }
            if (dts != "" && dts != "N/A") {
                x=dts+0.0
                if (have_dts && x+eps < last_dts) {
                    printf "%s DTS regression packet %d: %.9f < %.9f\n", name,n,x,last_dts > "/dev/stderr"
                    bad=1
                }
                last_dts=x; have_dts=1
            }
            if (dur != "" && dur != "N/A" && (dur+0.0) <= 0.0) {
                printf "%s non-positive duration packet %d: %s\n", name,n,dur > "/dev/stderr"
                bad=1
            }
            ++n
        }
        END {
            if (n == 0 || !have_pts || !have_dts) bad=1
            printf "%s_packets=%d\n",name,n
            exit bad ? 1 : 0
        }
    ' "$file"
}

video_packet_count=$(check_packets "$VIDEO_PACKETS" video)
audio_packet_count=$(check_packets "$AUDIO_PACKETS" audio)

duration_delta_ms=$(awk -v v="$vduration" -v a="$aduration" '
    BEGIN {
        if (v=="" || a=="" || v=="N/A" || a=="N/A") exit 2
        d=(v+0.0)-(a+0.0); if (d<0) d=-d
        printf "%.3f",d*1000.0
    }
') || { echo "stream duration unavailable" >&2; exit 1; }

awk -v d="$duration_delta_ms" -v limit="$MAX_AV_DURATION_DELTA_MS" \
    'BEGIN { exit ((d+0.0) <= (limit+0.0)) ? 0 : 1 }' || {
    echo "network A/V duration delta ${duration_delta_ms} ms exceeds ${MAX_AV_DURATION_DELTA_MS} ms" >&2
    exit 1
}

if ! ffmpeg -hide_banner -v error -i "$INPUT" \
        -map 0:v:0 -map 0:a:0 -f null - > "$DECODE_LOG" 2>&1; then
    cat "$DECODE_LOG" >&2
    exit 1
fi

{
    echo "input=$INPUT"
    echo "container=mpegts"
    echo "video_codec=$vcodec"
    echo "audio_codec=$acodec"
    echo "audio_sample_rate=$asample_rate"
    echo "audio_channels=$achannels"
    echo "video_time_base=$vtime_base"
    echo "audio_time_base=$atime_base"
    echo "video_duration=$vduration"
    echo "audio_duration=$aduration"
    echo "av_duration_delta_ms=$duration_delta_ms"
    echo "$video_packet_count"
    echo "$audio_packet_count"
    echo "decode_errors=0"
    echo "v8_4_network_capture_validation=PASS"
} | tee "$SUMMARY"

echo "streams=$STREAMS"
echo "video_packets=$VIDEO_PACKETS"
echo "audio_packets=$AUDIO_PACKETS"
echo "decode_log=$DECODE_LOG"
echo "summary=$SUMMARY"
